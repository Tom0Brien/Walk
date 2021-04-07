function output = evalFootGait(stepLength,stepHeight,numPoints)
    
    period = numPoints/3; 
    for t = 0:numPoints/3
        trajectory(:,t+1) = [0;0;(t)/period*stepHeight];
    end
    for t = 0:numPoints/3
        trajectory(:,(numPoints/3)+t+1) = [(t)/period*stepLength;0;stepHeight];
    end
    for t = 0:numPoints/3
        trajectory(:,2*(numPoints/3)+t+1) = [stepLength;0;stepHeight-(t)/period*stepHeight];
    end
    output = trajectory;
end