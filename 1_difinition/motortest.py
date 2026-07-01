import time
import motordrive


def main():
    try:
        print("========== DRV8411A Motor Test Start ==========")

        print("Setup Motors...")
        motordrive.setup_motors()
        time.sleep(1)

        print("Forward: w")
        motordrive.move("w", 0.3, 2.0, enable_stack_check=False)
        time.sleep(1)

        print("Backward: s")
        motordrive.move("s", 0.3, 2.0, enable_stack_check=False)
        time.sleep(1)

        print("Turn Left: a")
        motordrive.move("a", 0.3, 2.0, enable_stack_check=False)
        time.sleep(1)

        print("Turn Right: d")
        motordrive.move("d", 0.3, 2.0, enable_stack_check=False)
        time.sleep(1)

        print("Back Left: q")
        motordrive.move("q", 0.3, 2.0, enable_stack_check=False)
        time.sleep(1)

        print("Back Right: e")
        motordrive.move("e", 0.3, 2.0, enable_stack_check=False)
        time.sleep(1)

        print("========== Motor Test Finished ==========")

    except KeyboardInterrupt:
        print("\nTest interrupted.")
        motordrive.stop()

    except Exception as e:
        print(f"Error: {e}")
        motordrive.stop()

    finally:
        motordrive.cleanup()
        print("Cleanup finished.")


if __name__ == "__main__":
    main()
