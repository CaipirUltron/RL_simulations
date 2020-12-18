import numpy as np
import math

class QLearning:

    def  __init__(self, robot):
        self.robot = robot

        #Q_table(angle, line_of_sight, goal, action)
        #initialize Qtable with zeros
        self.Q_table = np.zeros( (8,8,2,3) )
        print(self.Q_table)
        self.Sangle=0




    def calculate_Sangle(self):         #calcular estado do erro do angulo entre frente do robot e posicao do goal
        self.angle = self.robot.angle       #working
        
       


        xG = int(self.robot.world.goal_position[0])
        yG = int(self.robot.world.goal_position[1])      #working

        curr_x = int(self.robot.position[0])
        curr_y = int(self.robot.position[1])    #working

        delta_x = int(xG) - int(curr_x)
        delta_y = -(int(yG) - int(curr_y))     #working

        print("xG: ", xG, "yG: ", yG)
        print("curr_x: ", curr_x, "curr_y: ", curr_y)
        print("delta y ", delta_y,"delta x ", delta_x)

        

        

        
        theta_goal = math.atan2(int(delta_y),int(delta_x))  #calcula angulo para o goal. Atencao: atan retorna de -pi/2 a pi/2 e nao resolve divisao por 0

        sin = math.sin(math.radians(self.robot.angle))
        cos = math.cos(math.radians(self.robot.angle))
        robot_angle = math.atan2(sin,cos)

        print("robot angle: ", robot_angle)

        theta_goal = -(math.degrees(theta_goal) - math.degrees(robot_angle))     #converte para graus

        print("theta_goal ", theta_goal)


        angle_array_positive = np.array([45, 90, 135, 180]) - 22.5
        angle_array_negative = np.array([-45, -90, -135, -180]) + 22.5

        if(theta_goal>=angle_array_negative[0] and theta_goal<angle_array_positive[0]):
            self.Sangle=0
        elif(theta_goal>=angle_array_positive[0] and theta_goal<angle_array_positive[1]):
            self.Sangle=1
        elif(theta_goal>=angle_array_positive[1] and theta_goal<angle_array_positive[2]):
            self.Sangle=2
        elif(theta_goal>=angle_array_positive[2] and theta_goal<angle_array_positive[3]):
            self.Sangle=3
        elif(theta_goal>=angle_array_positive[3] or theta_goal<angle_array_negative[3]):
           self.Sangle=4
        elif(theta_goal>=angle_array_negative[3] and theta_goal<angle_array_negative[2]):
            self.Sangle=5
        elif(theta_goal>=angle_array_negative[2] and theta_goal<angle_array_negative[1]):
            self.Sangle=6
        elif(theta_goal>=angle_array_negative[1] and theta_goal<angle_array_negative[0]):
            self.Sangle=7

        print("Sangle: ",self.Sangle)

        

        return self.Sangle


    def calculate_Stheta_obs(self):             #calcular estado da line of sight
        line_of_sight = self.robot.line_of_sight()

        #1 deve ser esquerda e -1 a direita
        if(line_of_sight[1]==0 and line_of_sight[0]==0 and line_of_sight[-1]==0) :
            self.Stheta_obs=0
        elif(line_of_sight[1]==1 and line_of_sight[0]==0 and line_of_sight[-1]==0): 
            self.Stheta_obs=1
        elif(line_of_sight[1]==0 and line_of_sight[0]==1 and line_of_sight[-1]==0): 
            self.Stheta_obs=2
        elif(line_of_sight[1]==0 and line_of_sight[0]==0 and line_of_sight[-1]==1): 
            self.Stheta_obs=3
        elif(line_of_sight[1]==1 and line_of_sight[0]==1 and line_of_sight[-1]==0): 
            self.Stheta_obs=4        
        elif(line_of_sight[1]==1 and line_of_sight[0]==0 and line_of_sight[-1]==1):
            self.Stheta_obs=5
        elif(line_of_sight[1]==0 and line_of_sight[0]==1 and line_of_sight[-1]==1):
            self.Stheta_obs=6
        elif(line_of_sight[1]==1 and line_of_sight[0]==1 and line_of_sight[-1]==1):
            self.Stheta_obs=7

        return self.Stheta_obs


    def calculate_Sgoal(self):      #calcular estado goal
        check_goal = self.robot.check_goal_reached()

        if(check_goal==True):
            return 1
        else:
            return 0


    def calculate_reward(self):
        self.reward=0
        if(self.robot.check_goal_reached()):
            self.reward = 10
        elif(self.robot.check_wall_collision()):
            self.reward = -1
        else:
            self.reward = 0

        
        error_angle = self.calculate_Sangle()

        if(error_angle==1 or error_angle==7):
            self.reward+=-1
        elif(error_angle==2 or error_angle==6):
            self.reward+=-2
        elif(error_angle==3 or error_angle==5):
            self.reward+=-3
        elif(error_angle==4):
            self.reward+=-4

        return self.reward





    def current_state(self):
        self.curr_state_angle = self.calculate_Sangle()
        self.curr_state_obs = self.calculate_Stheta_obs()
        self.curr_state_goal = self.calculate_Sgoal()
        return
    

    def next_state(self):
        self.next_state_angle = self.calculate_Sangle()
        self.next_state_obs = self.calculate_Stheta_obs()
        self.next_state_goal = self.calculate_Sgoal()
        return


    def choose_action(self):
        arr = np.array([0, 0, 0, 0])
        
        i=0
        while(i<3):  #passar valor das acoes do estado atual para o vetor auxiliar
            arr = self.Q_table[self.curr_state_angle, self.curr_state_obs, self.curr_state_goal, i]
            i+=1
        
        self.best_action = np.argmax(arr)             #ve qual coluna com maior valor

        if(self.best_action==0):
            self.robot.go_left()
        elif(self.best_action==1):
            self.robot.go_forward()
        elif(self.best_action==2):
            self.robot.go_right()

        self.action_taken = self.best_action  #para saber qual acao tomou
        return


    def update_Qvalue(self):
        self.alpha = 1
        self.gama = 0.7

        i=0
        while(i<3):  #passar valor das acoes do estado futuro para o vetor auxiliar
            arr = self.Q_table[self.next_state_angle, self.next_state_obs, self.next_state_goal, i]
            i+=1
        max_Qvalue = np.max(arr)    #ve qual o qvalue max para estado futuro
        
        self.Q_table[self.curr_state_angle, self.curr_state_obs, self.curr_state_goal, self.action_taken] += self.alpha*( self.calculate_reward() + self.gama*max_Qvalue - self.Q_table[self.curr_state_angle, self.curr_state_obs, self.curr_state_goal, self.action_taken] )

