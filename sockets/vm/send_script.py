#!usrbinpython3

import sys
import rospy
sys.path.append('homekendows_tmdriversrc')
from tm_msgs.msg import 
from tm_msgs.srv import 

import socket
import json
#import queue



def send_script(script)
    rospy.wait_for_service('tm_driversend_script')
    try
        pr = rospy.ServiceProxy('tm_driversend_script', SendScript)
#        print(SendScript)
        resp = pr('0', script)
        if resp.ok
            rospy.loginfo('send_script success.')
        else
            rospy.loginfo('send_script failed!')
    except rospy.ServiceException as e
        print(Service call failed %s%e)

def bool_string(my_bool)
    if my_bool
        return 'true'
    return 'false'

def PTP_CPP(x, y, z, rx, ry, rz, speed_percent, till_max_speed, blend, disable_precision)
    '''
    Parameters
    ----------
    x, y, z, rx, ry, rz  float
        Cartesian coordinate of target.
    speed_percent  int
        Movement speed percentage w.r.t. maximum speed setting of the robot.
    till_max_speed  int
        Number of ms taken to accelerate to the desired speed.
    blend  int
        Trajectory blend percentage.
    disable_precision  bool
        Whether to disable precise positioning.
    
    Returns
    -------
    string
        The script send via send_script.
    '''
    return 'PTP(CPP,%f,%f,%f,%f,%f,%f,%d,%d,%d,%s)'% 
        (x, y, z, rx, ry, rz, 
        speed_percent, till_max_speed, blend, bool_string(disable_precision))


HOST = 10.0.3.2
PORT = 7004
RECV_TIMEOUT_SEC = 2

def main()
    nh = rospy.init_node('arm')
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    print(connected)
    
    s.settimeout(0.01)
    try
        indata = s.recv(1024)
        print('clear first recv ' + indata.decode() + '')
    except socket.timeout
        print(nothing to be cleared)
    s.settimeout(RECV_TIMEOUT_SEC)

    #send_script(PTP_CPP( 
    #                , 
    #                , 
    #                800, 50, 100, False))
    #q = queue.Queue()
    buffer = 
    while not rospy.is_shutdown()
        try
            indata = s.recv(1024)
            instring = indata.decode()
            buffer = buffer + instring
            #print('buffer')
            #print(buffer)
            i = buffer.find(@)
            if i = 0
                #print('substring')
                #print(buffer[i])
                p = json.loads(buffer[i])
                print('parsed')
                print(p)
                send_script(PTP_CPP( 
                    p['x'], p['y'], p['z'], 
                    p['rx'], p['ry'], p['rz'], 
                    800, 50, 100, False))
                buffer = buffer[i+1]
        except json.decoder.JSONDecodeError
            print(json decoding interrupted)
            continue
        except socket.timeout
            # Yield regularly
            print(no data in %d seconds%RECV_TIMEOUT_SEC)
            continue

#    send_script(PTP_CPP(417, -122, 296, -68, 89, -158, 300, 50, 0, False))
#    send_script(PTP_CPP(417, -122, 396, -68, 89, -158, 300, 50, 0, False))
#    send_script(PTP_CPP(417, -122, 296, -68, 89, -158, 300, 50, 0, False))
    print(shutting down...)
    s.close()
    rospy.signal_shutdown(finished)

if __name__ == '__main__'
    main()