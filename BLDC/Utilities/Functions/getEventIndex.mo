within BLDC.Utilities.Functions;
function getEventIndex "get first index that triggered event"
  extends Modelica.Icons.Function;
  input Integer m "Max. index";
  input Boolean u[m] "Boolean vector";
  input Boolean pre_u[m] "pre of Boolean vector";
  output Integer ke "Index that triggered event";
algorithm
  ke:=m + 1;
  for k in 1:m loop
    if u[k] and not pre_u[k] then
      ke:=k;
      return;
    end if;
  end for;
  annotation (Documentation(info="<html>
<p>
Returns the index that triggerd the event inside a when-clause:
<pre>
  parameter Integer m=3;
  Boolean u[m];
  Integer k;
  when edge(u) then
    k=getEventIndex(m, u, pre(u));
  end when;
</pre>
If no event was triggered, <code>k=m + 1</code> is returned.
</p>
</html>"));
end getEventIndex;
