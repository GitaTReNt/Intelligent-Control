classdef PriorityQueue
    properties
        elements
    end

    methods
        function self = PriorityQueue()
            self.elements = {};
        end

        function insert(self, element, priority)
            self.elements{end + 1} = {priority, element};
            self.elements = sortrows(self.elements, 1);
        end

        function [element, priority] = pop(self)
            element = self.elements{1}{2};
            priority = self.elements{1}{1};
            self.elements(1) = [];
        end

        function isempty = isempty(self)
            isempty = isempty(self.elements);
        end
    end
end
