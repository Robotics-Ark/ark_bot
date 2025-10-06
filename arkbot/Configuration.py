#!/usr/bin/env python3

from servopkg.bus import ServoBus

def scan_servos(port: str, baudrate: int, max_id: int = 20):
    bus = ServoBus(port, baudrate)
    sdk = bus.sdk
    found = []
    try:
        for sid in range(0, max_id + 1):
            model, comm, err = sdk.ping(sid)
            if comm == 0 and err == 0 and model is not None:
                print(f"Servo found at ID {sid} (model {model})")
                found.append(sid)
    finally:
        bus.close()
    return found

def prompt_int(prompt: str, lo: int, hi: int) -> int:
    while True:
        s = input(prompt).strip()
        try:
            v = int(s)
            if lo <= v <= hi:
                return v
            print(f"Enter a number between {lo} and {hi}.")
        except ValueError:
            print("Please enter a valid integer.")

if __name__ == "__main__":
    DEVICENAME = "COM7"       # or "/dev/ttyUSB0"
    BAUDRATE   = 1_000_000

    # 1) Scan
    ids = scan_servos(DEVICENAME, BAUDRATE)
    if ids:
        print("\nConnected servo IDs:", ids)
    else:
        print("\nNo servos detected.")

    # --- NEW: optional midpoint action on all detected IDs ---
    if ids:
        mid_ans = input("Press 'm' to set ALL detected servos to midpoint (or Enter to skip): ").strip().lower()
        if mid_ans == "m":
            bus = ServoBus(DEVICENAME, BAUDRATE)
            sdk = bus.sdk
            try:
                for sid in ids:
                    ok = sdk.DefineMiddle(sid)
                    if ok:
                        print(f"✅ Midpoint defined on ID {sid}")
                    else:
                        print(f"❌ Failed to define midpoint on ID {sid}")
            finally:
                bus.close()
        print()  # spacer
    # ---------------------------------------------------------

    # 2) Ask to change ID
    ans = input("Change ID? (y/n): ").strip().lower()
    if ans != "y":
        print("Ok")
        raise SystemExit(0)

    current_id = prompt_int("Enter current ID to change: ", 0, 253)
    new_id     = prompt_int("Enter NEW ID: ", 0, 253)

    # 3) Open bus once and change via SDK
    bus = ServoBus(DEVICENAME, BAUDRATE)
    sdk = bus.sdk
    try:
        # Optional: sanity check the current ID exists
        _, comm, err = sdk.ping(current_id)
        if comm != 0 or err != 0:
            print(f"ID {current_id} is not responding; aborting.")
            raise SystemExit(1)

        res = sdk.ChangeID(current_id, new_id)
        if res is None:
            print(f"ID changed to {new_id}. Verifying...")
            _, comm2, err2 = sdk.ping(new_id)
            if comm2 == 0 and err2 == 0:
                print(f"Verified: servo responds at ID {new_id}.")
            else:
                print("⚠️ ID write may have succeeded, but ping to new ID failed.")
        else:
            # ChangeID returns an error string on failure
            print("Failed to change ID:", res)
    finally:
        bus.close()
