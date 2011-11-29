function st = yaml2struct(yml)
% This function converts a yaml string into a struct
% yml is a yaml formatted string 
%
%  %======================================================================
%{
		Copyright (c) 2011
		This program is a result of a joined cooperation of Energocentrum
		PLUS, s.r.o. and Czech Technical University (CTU) in Prague.
        The program is maintained by Energocentrum PLUS, s.r.o. and
        licensed under the terms of MIT license. Full text of the license
        is included in the program release.
		
        Author(s):
		Jiri Cigler, Dept. of Control Engineering, CTU Prague 
		Jan  Siroky, Energocentrum PLUS s.r.o.
		
        Implementation and Revisions:

        Auth  Date        Description of change
        ----  ---------   -------------------------------------------------
        jc    01-Mar-11   First implementation
        jc    02-Mar-11   .jar package initialization moved to external fun
        jc    18-Mar-11   Warning added when imported file not found
%}
%
% adapted by Jordan Brindza, University of Pennsylvania, 2011
%======================================================================

  InitYaml();

  import('org.yaml.snakeyaml.Yaml');

  yamlreader = Yaml();
  jymlobj = yamlreader.load(yml);

  st = Hash2Struct(jymlobj);

end % end of function


