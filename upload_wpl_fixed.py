#!/usr/bin/env python3
from pathlib import Path
import argparse
import time
from pymavlink import mavutil


def parse_wpl_file(path: Path):
    lines = [ln.strip() for ln in path.read_text(encoding="utf-8").splitlines() if ln.strip()]

    if not lines:
        raise ValueError("Файл пустой")

    if not lines[0].startswith("QGC WPL "):
        raise ValueError("Первая строка должна начинаться с 'QGC WPL '")

    raw_items = []
    for line_no, line in enumerate(lines[1:], start=2):
        parts = line.split()
        if len(parts) != 12:
            raise ValueError(f"Строка {line_no}: ожидалось 12 полей, получено {len(parts)}")

        raw_items.append({
            "seq": int(parts[0]),
            "current": int(parts[1]),
            "frame": int(parts[2]),
            "command": int(parts[3]),
            "p1": float(parts[4]),
            "p2": float(parts[5]),
            "p3": float(parts[6]),
            "p4": float(parts[7]),
            "lat": float(parts[8]),
            "lon": float(parts[9]),
            "alt": float(parts[10]),
            "autocontinue": int(parts[11]),
        })

    # Частый случай для QGC WPL / Mission Planner:
    # первая строка после заголовка — home-like запись, не обычный waypoint
    if raw_items:
        first = raw_items[0]
        if (
            first["seq"] == 0
            and first["current"] == 1
            and first["frame"] == 0
            and first["command"] == 16
        ):
            print("[INFO] Первая строка распознана как home-like, пропускаю её")
            raw_items = raw_items[1:]

    # Нормализуем seq заново
    for i, item in enumerate(raw_items):
        item["seq"] = i
        item["current"] = 0

    return raw_items




def connect_master(master_str: str, baud: int, timeout: float):
    print(f"[INFO] Подключаюсь к {master_str}")

    if master_str.startswith("/dev/"):
        master = mavutil.mavlink_connection(
            master_str,
            baud=baud,
            source_system=255,
            autoreconnect=True,
        )
    else:
        master = mavutil.mavlink_connection(
            master_str,
            source_system=255,
            autoreconnect=True,
        )

    hb = master.wait_heartbeat(timeout=timeout)
    if hb is None:
        raise TimeoutError("HEARTBEAT не пришёл")

    print(f"[OK] HEARTBEAT: system={master.target_system}, component={master.target_component}")

    # Если target_component остался 0, жёстко целимся в автопилотный компонент 1
    if master.target_component == 0:
        master.target_component = 1
        print("[INFO] target_component был 0, устанавливаю 1")

    return master


def build_mission_item_int(master, item):
    return mavutil.mavlink.MAVLink_mission_item_int_message(
        master.target_system,
        master.target_component,
        item["seq"],
        item["frame"],
        item["command"],
        item["current"],
        item["autocontinue"],
        item["p1"],
        item["p2"],
        item["p3"],
        item["p4"],
        int(round(item["lat"] * 1e7)),
        int(round(item["lon"] * 1e7)),
        item["alt"],
    )


def build_mission_item(master, item):
    return mavutil.mavlink.MAVLink_mission_item_message(
        master.target_system,
        master.target_component,
        item["seq"],
        item["frame"],
        item["command"],
        item["current"],
        item["autocontinue"],
        item["p1"],
        item["p2"],
        item["p3"],
        item["p4"],
        item["lat"],
        item["lon"],
        item["alt"],
    )


def wait_ack(master, timeout: float, label: str):
    deadline = time.time() + timeout
    while True:
        msg = master.recv_match(type=["MISSION_ACK"], blocking=True, timeout=1)
        if msg is None:
            if time.time() > deadline:
                raise TimeoutError(f"Таймаут ожидания MISSION_ACK ({label})")
            continue

        result = getattr(msg, "type", None)
        if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"[OK] MISSION_ACK ({label}) = ACCEPTED")
            return msg

        raise RuntimeError(f"MISSION_ACK ({label}) вернул код {result}")


def clear_mission(master, timeout: float):
    print("[INFO] Очищаю старую миссию")
    master.mav.mission_clear_all_send(
        master.target_system,
        master.target_component,
    )
    wait_ack(master, timeout, "clear_all")


def upload_mission(master, items, timeout: float):
    print(f"[INFO] Загружаю {len(items)} точек")

    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        len(items),
    )

    deadline = time.time() + timeout
    sent_any = False

    while True:
        msg = master.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
            blocking=True,
            timeout=1,
        )

        if msg is None:
            if time.time() > deadline:
                raise TimeoutError("Таймаут во время upload")
            continue

        mtype = msg.get_type()

        if mtype == "MISSION_REQUEST_INT":
            seq = int(msg.seq)
            if not (0 <= seq < len(items)):
                raise RuntimeError(f"Некорректный seq={seq}")
            master.mav.send(build_mission_item_int(master, items[seq]))
            print(f"[SEND] INT item {seq}/{len(items)-1}")
            sent_any = True
            deadline = time.time() + timeout
            continue

        if mtype == "MISSION_REQUEST":
            seq = int(msg.seq)
            if not (0 <= seq < len(items)):
                raise RuntimeError(f"Некорректный seq={seq}")
            master.mav.send(build_mission_item(master, items[seq]))
            print(f"[SEND] item {seq}/{len(items)-1}")
            sent_any = True
            deadline = time.time() + timeout
            continue

        if mtype == "MISSION_ACK":
            result = getattr(msg, "type", None)
            if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                if not sent_any and len(items) > 0:
                    raise RuntimeError(
                        "Получен ACCEPTED без единого MISSION_REQUEST(_INT). "
                        "Это не похоже на реальную загрузку миссии."
                    )
                print("[OK] Upload завершён")
                return

            raise RuntimeError(f"MISSION_ACK (upload) вернул код {result}")


def verify_count(master, timeout: float):
    print("[INFO] Проверяю count чтением обратно")

    master.mav.mission_request_list_send(
        master.target_system,
        master.target_component,
    )

    deadline = time.time() + timeout
    while True:
        msg = master.recv_match(type=["MISSION_COUNT"], blocking=True, timeout=1)
        if msg is None:
            if time.time() > deadline:
                raise TimeoutError("Не удалось получить MISSION_COUNT обратно")
            continue

        print(f"[OK] На стороне автопилота mission_count={msg.count}")
        return msg.count


def main():
    ap = argparse.ArgumentParser(description="Загрузка QGC WPL 110 по MAVLink")
    ap.add_argument("file", help="Файл .waypoints/.txt")
    ap.add_argument("--master", required=True, help="Например tcp:127.0.0.1:5760 или /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=10.0)
    args = ap.parse_args()

    items = parse_wpl_file(Path(args.file))
    print(f"[INFO] В файле найдено {len(items)} точек")

    master = connect_master(args.master, args.baud, args.timeout)
    clear_mission(master, args.timeout)
    upload_mission(master, items, args.timeout)
    verify_count(master, args.timeout)


if __name__ == "__main__":
    main()
