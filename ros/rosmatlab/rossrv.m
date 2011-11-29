function [req resp] = rossrv(srv_type, isService)
% srv_type - string name of the rossrv type (ex. begginer_tutorials/AddTwoInts)
%             or the rosservice name (ex. /add_two_ints
% isService - logical indicating that the srv_type input is the rosservice name
%               and not the rossrv type
% return:
%   req - matlab struct version of the service request
%   resp - matlab struct version of the service response (optional)
% 
% Jordan Brindza, University of Pennsylvania, 2011

if (nargin < 2)
  isService = false;
end

% make unique fifo
fifo = sprintf('/tmp/rossrvFIFO%f', now);
while (exist(fifo, 'file'))
  fifo = sprintf('/tmp/rossrvFIFO%f', now);
end
status = system(['mkfifo ' fifo]);
if (status ~= 0)
  error(['Could not create FIFO: ' fifo]);
end

if isService
  % input is the actual service name
  system(['rossrv show `rosservice type ' srv_type '` >> ' fifo ' & 2>&1']);
else
  % input is the service message type
  system(['rossrv show ' srv_type ' >> ' fifo ' & 2>&1']);
end

% open fifo
fid = fopen(fifo, 'r');
cdata = native2unicode(fread(fid));
fclose(fid);
system(['rm -f ' fifo]);

msg = char(cdata(:)');
reqresp = regexp(msg, '---\n', 'split');

req = parse_rosmsg(reqresp{1});
if (nargout > 1)
  resp = parse_rosmsg(reqresp{2});
end

