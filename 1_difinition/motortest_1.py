import sys
import time

# bno055のインポートによるエラーや無限ループを回避するためのダミークラス
class DummyBNO055:
    def begin(self):
        return True
    def gyroscope(self):
        return [0.0, 0.0, 0.0]  # リストで返す
    def linear_acceleration(self):
        return [0.0, 0.0, 0.0]  # 追加

# モジュールキャッシュにダミーを登録して motordrive 内でのインポートを差し替える
sys.modules['bno055'] = type('bno055_module', (), {'BNO055': DummyBNO055})

# ダミー登録後に motordrive をインポート
import motordrive

def main():
    print("--- モーターテスト開始 ---")
    print("コマンドを入力してください (方向: w/s/a/d/q/e, 反転付与: r, 終了: exit または Ctrl+C)")
    
    try:
        while True:
            cmd = input("\nコマンド入力 > ").strip().lower()
            if not cmd or cmd == 'exit':
                print("テストを終了するよ。")
                break
            
            # 'r' が含まれていれば逆さま走行（is_inverted=True）と判定
            is_inv = 'r' in cmd
            d = cmd.replace('r', '')
            
            # 有効な方向コマンドか判定
            if d in ['w', 's', 'a', 'd', 'q', 'e']:
                print(f"移動実行: 方向='{d}', 逆さま判定={is_inv}")
                
                # powerは0.0~1.0の範囲で指定 (ここでは0.5、動作時間は2秒)
                stuck = motordrive.move(
                    direction=d, 
                    power=0.5, 
                    duration=2.0, 
                    is_inverted=is_inv, 
                    enable_stack_check=True
                )
                
                # スタックが検知された場合の解除コード実行
                if stuck == 1:
                    print(">> スタックを検知したよ。スタック解除動作を開始します。")
                    motordrive.check_stuck(stuck, is_inverted=is_inv)
            else:
                print("無効なコマンドだよ。w (前進), s (後退), a (左旋回), d (右旋回), q (左後退), e (右後退) で入力してね。")

    except KeyboardInterrupt:
        print("\n中断されたよ。")
    finally:
        motordrive.cleanup()

if __name__ == "__main__":
    main()
