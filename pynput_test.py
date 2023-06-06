from pynput import keyboard

done = False

def on_press(key):
    print(f"pressed {key}")

def on_release(key):
    print(f"released {key}")

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

while not done:
    pass
