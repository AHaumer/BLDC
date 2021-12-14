within BLDC.Utilities.Functions;
function addIndex "add to an index in a cyclic way"
  extends Modelica.Icons.Function;
  input Integer k "Index";
  input Integer a "Addend";
  input Integer m "Max. index";
  output Integer result "Successor";
algorithm
  assert(k>=1 and k<=m, "Index out of range");
  result:=1 + mod(k + a - 1, m);
  annotation (Documentation(info="<html>
<p><pre>
  result:= k + a;
</pre></p>
and bring the result to the range [1, m] in a cyclic way, i.e. m + 1 -> 1 and 0 -> m.
The addend can also be negative.
</html>"));
end addIndex;
