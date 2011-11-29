function msg = parse_rosmsg(str)

  function [mclass isArray arrayLen] = isclass(t)
    persistent classMap;
    if isempty(classMap)
      classMap = {{'int8',    'int8'},
                  {'int16',   'int16'}, 
                  {'int32',   'int32'}, 
                  {'int64',   'int64'}, 
                  {'uint8',   'uint8'}, 
                  {'uint16',  'uint16'}, 
                  {'uint32',  'uint32'},
                  {'uint64',  'uint64'},
                  {'float32', 'single'}, 
                  {'float64', 'double'}, 
                  {'bool',    'logical'}, 
                  {'string',  'char'}, 
                  {'time',    'uint32'}}; 
    end

    mclass = [];
    isArray = false;
    arrayLen = 0;

    for i = 1:length(classMap)
      if strncmp(t, classMap{i}{1}, length(classMap{i}{1}))

        mclass = classMap{i}{2};

        % is array?
        aind = regexp(t, '\[.*\]');
        if ~isempty(aind)
          isArray = true;
          arrayLen = str2num(t(aind+1:end-1));
          if isempty(arrayLen)
            arrayLen = 0;
          end
        end

        break;
      end
    end
  end


  function [st, vdefs, vlevels, dvals] = parse(st, vdefs, vlevels, dvals);
    % done?
    if length(vdefs) == 0
      return;
    end

    % variable definition
    vdef = vdefs{1};
    % variable level
    vlevel = vlevels{1};
    % default value
    dval = dvals{1};

    % remove them from the arrays
    vdefs(1) = [];
    vlevels(1) = [];
    dvals(1) = [];

    if length(vdef) > 1
      vtype = vdef{1}; 
      vname = vdef{2}; 

      % is it a base type?
      [mclass isArray arrayLen] = isclass(vtype);
      if ~isempty(mclass)
        val = [];
        if isempty(dval)
          if isArray
            val = cast(zeros(arrayLen,1), mclass);
          elseif strcmp(mclass, 'char')
            val = '';
          else
            val = cast(0, mclass);
          end
        else
          % the token includes the equal sign
          val = cast(eval(dval(2:end)), mclass);
        end

        st = setfield(st, vname, val);
      else
        % submessage
        % find extent
        submsgi_end = 1;

        while (vlevels{submsgi_end} > vlevel) 
          submsgi_end = submsgi_end + 1;
        end

        [subst, edefs, elevels, evals] = parse([], vdefs(1:submsgi_end-1), vlevels(1:submsgi_end-1), dvals(1:submsgi_end-1)); 
        st = setfield(st, vname, subst);

        % remove the used fields from the arrays
        vdefs(1:submsgi_end-1) = [];
        vlevels(1:submsgi_end-1) = [];
        dvals(1:submsgi_end-1) = [];
      end
    end

    [st, vdefs, vlevels, dvals] = parse(st, vdefs, vlevels, dvals);
  end

% split msg string by lines
lines = regexp(str, '\n', 'split');
% split each line by first equal sign
[defs, dvals] = cellfun(@(l) strtok(l, '='), lines, 'UniformOutput', false);
vlevels = cellfun(@(d) length(regexp(d, '  ', 'split')) - 1, defs, 'UniformOutput', false);
vdefs = cellfun(@(d) regexp(strtrim(d), ' ', 'split'), defs, 'UniformOutput', false);

msg = [];
[msg vdefs, vlevels, dvals] = parse(msg, vdefs, vlevels, dvals);

end
