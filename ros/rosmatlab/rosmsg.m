function msg = rosmsg(msg_type)
% msg_type - string name of the ros message (ex. 'geometry_msgs/Pose')
% return:
%   matlab struct representation of the ros message
% 
% Jordan Brindza, University of Pennsylvania, 2011

% make unique fifo
fifo = sprintf('/tmp/rosmsgFIFO%f', now);
while (exist(fifo, 'file'))
  fifo = sprintf('/tmp/rosmsgFIFO%f', now);
end
status = system(['mkfifo ' fifo]);
if (status ~= 0)
  error(['Could not create FIFO: ' fifo]);
end

system(['rosmsg show ' msg_type ' >> ' fifo ' & 2>&1']);

% open fifo
fid = fopen(fifo, 'r');
cdata = native2unicode(fread(fid));
fclose(fid);
system(['rm -f ' fifo]);

msg = parse_rosmsg(char(cdata(:)'));

