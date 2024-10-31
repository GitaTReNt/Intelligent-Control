classdef Policy < handle
    properties
        obstacle_map
        map_data
        path_data
        target_point
        evaluation_params
        velocity_increment
        angle_increment
        velocity_limit
        angle_limit
        time_increment
        agent_info
        system_info
        sensor_info
        current_target
    end

    methods
        function self = Policy()
            self.path_data = [];
            self.target_point = [];
            abspath = utils('abspath');
            self.loadIni(abspath('sys.ini'));
            self.evaluation_params = [0.33, 0.05, -0.2, 0.1];%距离 朝向 目标距离 速度
            self.time_increment = self.system_info.st;
            self.velocity_limit = self.agent_info.usat;
            self.angle_limit = self.agent_info.vsat;
            self.velocity_increment = self.velocity_limit / 5;
            self.angle_increment = self.angle_limit / 5;
            self.obstacle_map = zeros(50, 50);
        end

        function action = action(self, observation)
            self.updateMap(observation.scanMap);
            if isempty(self.path_data)
                self.initializePath(observation);
            end

            state = [observation.agent.x, observation.agent.y];
            self.updateCurrentTarget(state);
            action = self.dynamicWindowApproach(observation, self.current_target);
        end

        function updateMap(self, scanMap)
            self.obstacle_map(scanMap == 1) = 1;
            self.map_data = self.obstacle_map;
            self.map_data(scanMap >= 1) = scanMap(scanMap >= 1);
        end

        function initializePath(self, observation)
            start = [observation.startPos.x, observation.startPos.y];
            goal = [observation.endPos.x, observation.endPos.y];
            astar = AStar(start, goal, observation.map);
            self.path_data = astar.AStarSearch(astar.obstacles, astar.map);
            self.target_point = self.getTarget(self.path_data);
            self.current_target = self.target_point(1, :);
            self.target_point(1, :) = [];
        end

        function updateCurrentTarget(self, state)
            dx = self.current_target(1) - state(1);
            dy = self.current_target(2) - state(2);
            if sqrt(dx^2 + dy^2) <= 4 && ~isempty(self.target_point)
                self.current_target = self.target_point(1, :);
                self.target_point(1, :) = [];
            end
        end

        function target = getTarget(self, path)
            len = size(path, 1);
            theta = atan2(diff(path(:, 2)), diff(path(:, 1)));
            target = path([true; diff(theta) ~= 0; true], :);
        end

        function action = dynamicWindowApproach(self, observation, target)
            evalDB = self.evaluate(observation, target);
            if isempty(evalDB)
                disp('no path');
                action = [0, 0];
                return;
            end
            evalDB = self.normalize(evalDB);
            [~, idx] = max(self.evaluateFevel(evalDB));
            action = evalDB(idx, 1:2);
        end

        function evalDB = evaluate(self, observation, target)
            evalDB = [];
            for vt = self.velocity_limit:-self.velocity_increment:-self.velocity_limit + 0.2
                for theta_t = -self.angle_limit:self.angle_increment:self.angle_limit
                    agent = self.initializeAgent(observation);
                    agent.step([0, self.time_increment], [vt, theta_t]);
                    if self.checkCollision(agent)
                        continue;
                    end
                    state = agent.TF();
                    evalDB = [evalDB; self.computeEvaluationMetrics(state, target, vt, theta_t)];
                end
            end
        end

        function agent = initializeAgent(self, observation)
            agent = Agent(observation.agent.x, observation.agent.y, observation.agent.h, ...
                          self.sensor_info.scanRange, self.sensor_info.scanAngle, 1);
            agent.setSatLevel([self.velocity_limit, self.angle_limit]);
        end

        function collision = checkCollision(self, agent)
            occupy_map = self.clip(agent.occupyMap(), size(self.map_data));
            collision = any(self.map_data(sub2ind(size(self.map_data), occupy_map(1, :), occupy_map(2, :))) >= 1);
        end

        function metrics = computeEvaluationMetrics(self, state, target, vt, theta_t)
            state(3) = self.standardH(state(3));
            distance = self.calcDistance(state);
            target_distance = self.calcDistanceToTarget(state, target);
            heading = self.calcHeading(state, target);
            metrics = [vt, theta_t, distance, heading, target_distance, vt];
        end

        function eval = normalize(self, eval)
            eval(:, 3:6) = bsxfun(@rdivide, eval(:, 3:6), max(abs(eval(:, 3:6)), [], 1));
        end

        function fevel = evaluateFevel(self, evalDB)
            fevel = evalDB(:, 3:6) * self.evaluation_params';
        end

        function h = standardH(self, r)
            h = mod(r, 2 * pi);
            if h > pi
                h = h - 2 * pi;
            end
        end

        function dist = calcDistance(self, state)
            [obx, oby] = find(self.map_data >= 1);
            if isempty(obx)
                dist = 8;
                return;
            end
            distances = sqrt((state(1) - obx).^2 + (state(2) - oby).^2);
            dist = min(distances);
        end

        function heading = calcHeading(self, state, goal)
            dx = goal(1) - state(1);
            dy = goal(2) - state(2);
            goal_theta = atan2(dy, dx);
            heading = 180 - abs(goal_theta - state(3)) * 180 / pi;
        end

        function target_dist = calcDistanceToTarget(self, state, target)
            target_dist = -sqrt((state(1) - target(1))^2 + (state(2) - target(2))^2);
        end

        function loadIni(self, file)
            I = INI('File', file);
            I.read();
            sections = I.get('Sections');
            for i = 1:length(sections)
                switch sections{i}
                    case 'RangeFinder'
                        self.sensor_info = I.get('RangeFinder');
                    case 'Agent'
                        self.agent_info = I.get('Agent');
                    case 'System'
                        self.system_info = I.get('System');
                end
            end
        end

        function crod = clip(self, crods, siz)
            crod = crods(:, crods(1, :) > 0 & crods(2, :) > 0 & crods(1, :) <= siz(1) & crods(2, :) <= siz(2));
        end
    end
end
