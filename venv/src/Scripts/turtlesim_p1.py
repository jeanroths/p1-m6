import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TPose
from collections import deque

MAX_DIFF = 0.1 # variável para verificar se a toruga chegou no ponto desejado
#classe para representar a pose da toruga
class Pose(TPose):
    def __init__(self, x=0.0, y=0.0, theta = 0.0): #construtor da classe
        super().__init__(x=x, y=y, theta=theta)
        
    def __repr__(self):
        return f"(x={self.x:.2f}, y={self.y:.2f})"
    
    def __add__(self, other): #Sempre que ver '+' executar esse método
        self.x += other.x
        self.y += other.y
        return self
    
    def __sub__(self, other): #Sempre que ver '-' executar esse método
        self.x -= other.x
        self.y -= other.y
        return self
    
    def __eq__(self, other): #Fazer overload do operador ==
        return abs(self.x - other.x) <= MAX_DIFF and abs(self.y - other.y) <= MAX_DIFF
    
class MissionControl(deque): # Classe em que os pontos para a toruga seguir são passados
    
    def __init__(self): 
        super().__init__()
        # lista de pontos que a toruga deve seguir
        self.enqueue(Pose(0.0, 0.5))
        self.enqueue(Pose(0.5, 0.0))
        self.enqueue(Pose(0.0, 0.5))
        self.enqueue(Pose(0.5, 0.0))
        self.enqueue(Pose(0.0, 1.0))
        self.enqueue(Pose(1.0, 0.0))


    def enqueue(self, x):
        """Método para adicionar novos pontos ao fim da fila."""
        super().append(x)
    
    def dequeue(self):
        """Método para retirar pontos do começo da fila."""
        return super().popleft()
    
class TurtleController(Node):
    def __init__(self, mission_control = MissionControl(), control_period=0.02):
        super().__init__('turtle_controller')

        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)

        self.timer_ = self.create_timer(control_period, self.control_callback)
        self.pose = Pose(x=-20.0) #pose atual da tartaruga
        self.mission_control = mission_control #cria uma instância da classe MissionControl
        self.setpoint = Pose(x=-20.0) #ponto atual que a tartaruga deve seguir

    def control_callback(self): 
        if self.pose.x == -20.0: # se a pose não foi iniciada, não fazer nada e aguardar pela pose
            self.get_logger().info("Aguardando pose...")
            return
        msg = Twist() #cria uma nova mensagem de velocidade
        x_diff = self.setpoint.x - self.pose.x #calcula a diferença entre o ponto desejado e o atual
        y_diff = self.setpoint.y - self.pose.y
        if self.pose == self.setpoint:
            msg.linear.x, msg.linear.y = 0.0, 0.0 #se a tartaruga já chegou no ponto desejado, não fazer nada
            self.update() #atualiza o ponto desejado
            self.get_logger().info(f"Chegou no ponto {self.setpoint}")
            exit()
        if abs(x_diff) > MAX_DIFF:
            msg.linear.x = 1.0 if x_diff > 0 else -1.0 #se a diferença for maior que o máximo, andar para frente ou para trás
        else:
            msg.linear.x = 0.0
        if abs(y_diff) > MAX_DIFF:
            msg.linear.y = 1.0 if y_diff > 0 else -1.0
        else:
            msg.linear.y = 0.0
        self.publisher_.publish(msg) #publica a mensagem de velocidade

    def update(self):
        self.setpoint = self.pose + self.mission_control.dequeue() #atualiza o ponto desejado
        self.pose = self.setpoint
        self.get_logger().info(f"A tartaruga está em {self.pose} e vai para {self.setpoint}")

    def pose_callback(self, msg):
         self.pose = Pose(x = msg.x, y = msg.y, theta= msg.theta) #atualiza a pose da tartaruga
         if self.setpoint.x == -20.0: #se a pose não foi iniciada, iniciar
             self.update()
             self.get_logger().info(f"A tartaruga está em: x={msg.x}, y={msg.y}, theta={msg.theta}")       
      
def main(args=None): #função principal
        rclpy.init(args=args)
        turtle_controller = TurtleController()
        rclpy.spin(turtle_controller)
        turtle_controller.destroy_node()
        rclpy.shutdown()

#Execução do programa
if __name__ == '__main__':
        main()