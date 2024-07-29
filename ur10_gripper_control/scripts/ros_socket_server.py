import socket
from threading import Thread, Event
import json
import signal

import rospy
from ur10_gripper_control.srv import MoveRobot, MoveRobotRequest
from geometry_msgs.msg import Pose

class Action:
    def __init__(self):
        self.pose = Pose()
        self.gripper = False

    def toString(self):
        print(self.pose.position.x, self.pose.position.y, self.pose.position.z)


class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(5)

        self.stop_event = Event()
        signal.signal(signal.SIGINT, self.__handle_sigint)

    def __handle_sigint(self, frame):
        self.stop_event.set()
        self.server.close()

    def listen_for_clients(self):
        rospy.loginfo("Socket server listening on %s:%s", self.host, str(self.port))
        while not self.stop_event.is_set():
            client, addr = self.server.accept()
            rospy.loginfo('Accepted Connection from: ' + str(addr[0]) + ':' + str(addr[1]))
            Thread(target=self.handle_client, args=(client, addr)).start()

    def handle_client(self, client_socket, address):
        size = 1024
        while not self.stop_event.is_set():
            try:
                data = client_socket.recv(size)
                rospy.loginfo('Received: ' + data.decode("utf-8") + ' from: ' + str(address[0]) + ':' + str(address[1]))
                
                actions = json.loads(data.decode("utf-8"))
                
                # Data will be [x, y] relative to camera coordinates, z and quaternion relative to robot base
                # Need to send back "task finish" when the robot did its task and came back to its init pos : [664.36, 728.00, -242.37,  0.9998099, 0.003164, -0.016769, -0.0094337 ] x, y, z, rx, ry, rz, w
        
                rospy.wait_for_service('/ur10_gripper_controller/MoveRobot')
                move_robot = rospy.ServiceProxy('/ur10_gripper_controller/MoveRobot', MoveRobot)

                poses = []
                grippers = []

                for action in actions:
                    a = Action()

                    # Assuming the JSON has 'pos_end_effector' as a list with 3 values for position
                    pos_end_effector = action["pos_end_effector"]
                    a.pose.position.x = pos_end_effector[0]
                    a.pose.position.y = pos_end_effector[1]
                    a.pose.position.z = pos_end_effector[2]
                    
                    # Assuming orientation is not provided, set default
                    a.pose.orientation.x = pos_end_effector[3]
                    a.pose.orientation.y = pos_end_effector[4]
                    a.pose.orientation.z = pos_end_effector[5]
                    a.pose.orientation.w = pos_end_effector[6]
                    
                    # Parse gripper state
                    if(action['gripper']):
                        a.gripper = True
                    
                    poses.append(a.pose)
                    grippers.append(a.gripper)

                # Create a request to the service
                response = move_robot(poses, grippers)
                
                # Check the result and print appropriate message
                if response.result:
                    rospy.loginfo("Result: %s", response.result)
                    # send getting after receiving from client
                    client_socket.sendall(json.dumps({"result" : str(response.result)}).encode())
                else:
                    client_socket.sendall(json.dumps({"result" : "fail to call service"}).encode())
                    rospy.logwarn("Service call failed")
                

            except socket.error:
                client_socket.close()
                return False


if __name__ == '__main__':
    rospy.init_node('ros_socket_server')

    HOST = rospy.get_param('~server_ip', "127.0.0.1")
    PORT = int(rospy.get_param('~server_port', '65500'))

    server = Server(HOST, PORT)

    server.listen_for_clients()

    rospy.spin()
