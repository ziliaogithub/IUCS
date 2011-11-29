function val = get_rosparam(param, mclass)
% param - string parameter name
% mclass - optional param class/type if known it will 
%           convert the output to the correct matlab class
% Jordan Brindza, University of Pennsylvania, 2011

% make unique fifo
fifo = sprintf('/tmp/rosparamFIFO%f', now);
while (exist(fifo, 'file'))
  fifo = sprintf('/tmp/rossrvFIFO%f', now);
end
status = system(['mkfifo ' fifo]);
if (status ~= 0)
  error(['Could not create FIFO: ' fifo]);
end

system(['rosparam get ' param ' >> ' fifo ' & 2>&1']);

% open fifo
fid = fopen(fifo, 'r');
cdata = native2unicode(fread(fid));
fclose(fid);
system(['rm -f ' fifo]);

valstr = char(cdata(:)');

if (nargin > 1)
  % convert the output based on the given class type
  if strcmp(mclass, 'struct')
    val = yaml2struct(valstr);
  elseif strcmp(mclass, 'logical')
    val = eval(valstr);
  elseif (strcmp(mclass, 'single') || strcmp(mclass, 'double') ...
          || strcmp(mclass, 'uint8') || strcmp(mclass, 'uint16') ...
          || strcmp(mclass, 'uint32') || strcmp(mclass, 'uint64') ...
          || strcmp(mclass, 'int8') || strcmp(mclass, 'int16') ...
          || strcmp(mclass, 'int32') || strcmp(mclass, 'int64'))
    val = cast(str2num(valstr), mclass);
  elseif isa(val, 'char')
    val = valstr;
  else
    error(sprintf('unsupported param class %s', class(val)));
  end
else
  val = valstr;
end

