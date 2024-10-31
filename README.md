# Introduction to Arena Challenge
Arena Challenge is a simulation environment written in Matlab code. The main requirement is that the challenger writes Matlab code to control a cart from start to finish in a map environment with randomly placed obstacles (see below). The cart is powered by a unicycle (explained below) and is equipped with a “radar” detector to detect obstacles ahead. The final score is assessed by the time it takes for the cart to reach the finish line and whether or not the cart hits the obstacle.

The challenger submits his/her designed control strategy, which will be tested on a randomized map and in different configurations to obtain a final score.

<div align="left">
<img src=https://gitee.com/coralab/ic-challenge/raw/master/Arena/pics/arena_preview.png width=60%/>
</div>


## 1. Dynamic model of the trolley
The dynamics of the trolley is modeled as shown in the following equation
<div align="left">
<img src=https://gitee.com/coralab/ic-challenge/raw/master/Arena/pics/unicycle.png width=30%/>
</div>


Where [x,y,$\theta$] are the current spatial position and orientation of the cart. The [u,v] are the control quantities of the cart. u is the “gas” pedal of the cart, which controls the forward speed of the cart; v is the “steering wheel” of the cart, which controls the speed of rotation of the cart. These are the only two physical quantities that the challenger can control when writing the code.

In addition, the simulation environment has set up a “saturation” mechanism for the cart (as shown in the figure below), i.e., the two control quantities of the cart passed by the challenger to the simulator will be limited to a certain range.

<div align="left">
<img src=https://gitee.com/coralab/ic-challenge/raw/master/Arena/pics/saturation.png width=30%/>
</div>


## 2. Radar Detector
There is a forward-facing radar sensor in front of the cart that detects whether there is an obstacle in front of it, and if there is an obstacle, it will inform the challenger about the obstacle. See the radar detection range indicated by the red triangle in front of the cart in the figure.

## 3. Observation（当前环境信息）

The simulation environment will inform the challenger about the simulation environment at regular intervals in the form of the Observation class, whose member variables contain  
```
agent		%当前小车信息  
scanMap	%当前雷达探测器探测到的信息  
t            	%当前所用时间  
collide    	%当前是否发生碰撞  
score      	%当前挑战者的分数  
startPos   	%小车起始位置  
endPos     	%小车目标位置  
map         	%全局地图  
```

## 4. Score
At the beginning of the challenge, the challenger will have a base score, if the challenger controls the cart and the obstacle collision will be deducted accordingly, if the challenger controls the cart out of the map range will be deducted accordingly (deductions weighted heavily), if the challenger controls the cart does not reach the end within the time limit, will be deducted accordingly for each second beyond the time limit.

## 5. Designing the control policy
The challenger needs to design and submit a Policy class file, which mainly completes the action function. action function passes in the parameter observation and passes out the action. the simulator will call the action at the characteristic time interval, and get the action, i.e., the control quantity [u,v], based on the policy designed by the challenger, so as to control the cart.
```
classdef Policy < handle
        function action=action(self,observation)
            if observation.collide
                action=[-10,rand(1)-0.5];
            else
                action=[10,rand(1)-0.5];
            end
        end
end
```

## 6. Main function
The following is the basic code of the Main function. main reads the system configuration file to configure the simulation environment and then enters the simulation loop. In the loop, the simulator performs physical simulation of Arena Challenge, calculates the information such as the position and state of the cart, whether there is a collision or not. And every time, it uses the Challenger to design the control strategy, and then acts the control strategy into the control of the cart.
```
env = Env('sys.ini');   %读取系统配置文件
policy=Policy();
if (env.succeed)
    observation = env.reset();
    while 1
        env.render();
        action=policy.action(observation); %调用挑战者的控制策略
        [observation,done,info]=env.step(action); %物理仿真
        disp(info);
        if(done)
            break;
        end
        wait(100);
    end
end
```

## 7. Simulation configuration file sys.ini
In order to facilitate the testing of the challenger, the challenger can use the simulation configuration file sys.ini to configure accordingly. For example, configure the start and end points of the cart, the saturation range of the cart control, and whether to record the running process of the game. See this file for details.
## 8. Challenge mode
In sys.ini, set globalview to 1, the challenger can get the global map information from observation at the beginning; if globalview is set to 0, the challenger will only get the local map information obtained by the sensors from observation at each system refresh;


# Submission

<http://www.rayliable.net/manage/account/login>

# Leader Board
<http://www.rayliable.net/manage/item/ranking>


