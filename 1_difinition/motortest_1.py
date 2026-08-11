import time
import motordrive_1

def main():
    print("モーターテストを開始します...")
    
    commands = ['w', 's', 'a', 'd', 'q', 'e']
    
    try:
        for cmd in commands:
            print(f"コマンド実行: {cmd}")
            motordrive_1.move(cmd)
            time.sleep(2)
            motordrive_1.stop()
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("テストを中断しました。")
    finally:
        motordrive_1.stop()
        print("モーターを停止して終了しました。")

if __name__ == "__main__":
    main()
