from evdev import InputDevice, categorize, ecodes
dev = InputDevice('/dev/input/event0')

print(dev)

for event in dev.read_loop():
    print(f"code: {event.code} type: {event.type} value: {event.value}")
