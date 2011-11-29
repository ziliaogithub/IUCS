function set_rosparam(param, val)
% param - string parameter name
% val - parameter value
% 
% Jordan Brindza, University of Pennsylvania, 2011

if isa(val, 'struct')
  valstr = sprintf('"%s"', struct2yaml(val));
elseif isa(val, 'logical')
  if val
    valstr = 'true';
  else
    valstr = 'false';
  end
elseif (isa(val, 'single') || isa(val, 'double'))
  valstr = sprintf('%f', val);
elseif (isa(val, 'uint8') || isa(val, 'uint16') ...
        || isa(val, 'uint32') || isa(val, 'uint64'))
  valstr = sprintf('%u', val);
elseif (isa(val, 'int8') || isa(val, 'int16') ...
        || isa(val, 'int32') || isa(val, 'int64'))
  valstr = sprintf('%d', val);
elseif isa(val, 'char')
  valstr = val;
else
  error(sprintf('unsupported param class %s', class(val)));
end

system(['rosparam set ' param ' ' valstr]);

