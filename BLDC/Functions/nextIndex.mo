within BLDC.Functions;
function nextIndex "returns successor of index in a cyclic way"
  extends Modelica.Icons.Function;
  input Integer m "Max. index";
  input Integer k "Index";
  output Integer ks "Successor";
algorithm
  assert(k>=1 and k<=m, "Index out of range");
  ks:=if k + 1 > m then 1 else k + 1;
  annotation (Documentation(info="<html>
<p><pre>
  successor:= if k+1 > m then 1 else k + 1;
</pre></p>
</html>"));
end nextIndex;
