import ROVER_socket as r
import CarControl as c

a = c.CarControl()
s = r.TCP_server()

try : 
    a.reset()
    while True :
        recv_msg = s.recv_string(100)
        print(recv_msg)
        if recv_msg == 'Close' :
            a.exit()
            s.close()
        elif recv_msg == '0' :
            a.stop()
        elif recv_msg == '1' :
            a.straight()
        elif recv_msg == 'w' :
            a.forward()
        elif recv_msg == 's' :
            a.backward()
        elif recv_msg == 'a' :
            a.turn_left()
        elif recv_msg == 'd' :
            a.turn_right()
        elif recv_msg == 'fast' :
            a.speed_up()
        elif recv_msg == 'slow' :
            a.speed_down()

except :
    a.exit()
    s.close()