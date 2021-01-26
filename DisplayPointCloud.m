arduinoObj = serialport("COM3",9600)

configureTerminator(arduinoObj,"CR/LF");

flush(arduinoObj);

arduinoObj.UserData = struct("x",[],"y",[],"z",[],"Count",1)

configureCallback(arduinoObj,"terminator",@readLidarData);

function readLidarData(src, ~)

data = readline(src);

src.UserData.x(end+1)=str2double(extractBetween(data,"x","y"));
src.UserData.y(end+1)=str2double(extractBetween(data,"y","z"));
src.UserData.z(end+1)=str2double(extractBetween(data,"z",strlength(data)));

src.UserData.Count = src.UserData.Count + 1;
if src.UserData.Count == 1600
    configureCallback(src, "off");
    ptCloud = pointCloud([src.UserData.x' src.UserData.y' src.UserData.z']);
    pcshow(ptCloud);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end
end