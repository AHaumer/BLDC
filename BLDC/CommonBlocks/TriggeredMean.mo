within BLDC.CommonBlocks;
block TriggeredMean "Calculate mean over period determined by trigger"
  extends Modelica.Blocks.Interfaces.SISO;
  import Modelica.Constants.eps;
  parameter Real x0=0 "Start value of integrator state";
  parameter Real y0=0 "Start value of output";
  parameter Boolean yGreaterOrEqualZero=false
    "= true, if output y is guaranteed to be >= 0 for the exact solution"
    annotation (Evaluate=true, Dialog(tab="Advanced"));
  Modelica.Blocks.Interfaces.BooleanInput trigger
    annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
protected
  Modelica.Units.SI.Time t0 "Last trigger instance";
  Real x "Integrator state";
  discrete Real y_last "Last sampled mean value";
initial equation
  pre(trigger)=false;
  t0 = time;
  x = x0;
  y_last = y0;
equation
  der(x) = u;
  when edge(trigger) then
    y_last = if noEvent((time - pre(t0))>eps) then
      if not yGreaterOrEqualZero then pre(x)/(time - pre(t0))
        else max(0.0, pre(x)/(time - pre(t0)))
      else pre(y_last);
    t0 = time;
    reinit(x, 0);
  end when;
  y = y_last;
  annotation (Documentation(info="<html>
<p>
This block calculates the mean of the input signal u over the period dtermined by the edges of the trigger:
</p>
<blockquote><pre>
 1   T
---- &int; u(t) dt
T-t0 t0
</pre></blockquote>
<p>
Note: The output is updated at each trigger instance.
</p>

<p>
If parameter <strong>yGreaterOrEqualZero</strong> in the Advanced tab is <strong>true</strong> (default = <strong>false</strong>),
then the modeller provides the information that the mean of the input signal is guaranteed
to be &ge; 0 for the exact solution. However, due to inaccuracies in the numerical integration scheme,
the output might be slightly negative. If this parameter is set to true, then the output is
explicitly set to 0.0, if the mean value results in a negative value.
</p>
</html>"),
         Icon(graphics={Text(
          extent={{-80,60},{80,20}},
          textString="mean")}));
end TriggeredMean;
