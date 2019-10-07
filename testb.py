import time
import rover_socket

gui_server = rover_socket.UDP_client(50010, 0, '192.168.5.2')
run = True
while run:
    gui_server.send_list([1])
    gui_receive = gui_server.recv_list(32768)
    print(gui_receive)
    time.sleep(0.1)
    # if gui_server.addr is not None:
        # print(gui_receive)
        # gui_server.send_list_back()
        