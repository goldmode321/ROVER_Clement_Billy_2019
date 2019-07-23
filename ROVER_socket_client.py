import ROVER_socket as r
from pynput import keyboard

c = r.TCP_client()

def on_press(key):
    try:
        print(key.char)
    except AttributeError:
        if key == keyboard.Key.up :
            print('Forward')
            c.send_string('w', 50000, '192.168.1.55')
        elif key == keyboard.Key.down :
            print('Backward')
            c.send_string('s', 50000, '192.168.1.55')
        elif key == keyboard.Key.right :
            print('Right')
            c.send_string('d', 50000, '192.168.1.55')
        elif key == keyboard.Key.left :
            print('left')
            c.send_string('a', 50000, '192.168.1.55')

def on_release(key):
    if key == keyboard.Key.esc:
        c.send_string('Close', 50000, '192.168.1.55')
        c.close()
        return False
    elif key == keyboard.Key.up or key == keyboard.Key.down:
        print('Stop')
        c.send_string('0', 50000, '192.168.1.55')
    elif key == keyboard.Key.right or key == keyboard.Key.left :
        print('Straight')
        c.send_string('1', 50000, '192.168.1.55')
    elif key == keyboard.Key.page_up:
        print('Speed up')
        c.send_string('fast', 50000, '192.168.1.55')
    elif key == keyboard.Key.page_down :
        print('Speed down')
        c.send_string('slow', 50000, '192.168.1.55')

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release,
        suppress=True) as listener:
    listener.join()

