from pynput import keyboard

def on_press(key):
    try:
        print(f"按下: {key.char}")
    except AttributeError:
        print(f"特殊按键: {key}")

def on_release(key):
    print(f"释放: {key}")
    if key == keyboard.Key.esc:  # 按下 Esc 键退出
        print("退出程序")
        return False

# 开始监听
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

