function resp = rosservice(service, req)
% service - string name of the ros service to call
% req - matlab struct version of the service request
% return:
%   resp - matlab struct version of the service response
% 
% Jordan Brindza, University of Pennsylvania, 2011

% construct yaml arguments
yml_args = ['"' struct2yaml(req) '"'];

% make unique fifo
fifo = sprintf('/tmp/rosserviceFIFO%f', now);
while (exist(fifo, 'file'))
  fifo = sprintf('/tmp/rosserviceFIFO%f', now);
end
status = system(['mkfifo ' fifo]);
if (status ~= 0)
  error(['Could not create FIFO: ' fifo]);
end

system(['rosservice call ' service ' ' yml_args ' >> ' fifo '& 2>&1']);

% open fifo
fid = fopen(fifo, 'r');
cdata = native2unicode(fread(fid));
fclose(fid);
system(['rm -f ' fifo]);

resp = [];
if (strcmp(cdata, '') == 0)
  resp = yaml2struct(cdata);
end

