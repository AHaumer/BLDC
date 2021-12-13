within BLDC.ToMSL;
block TriggeredRootMeanSquare
  "Calculate root mean square over period determined by trigger"
  extends Modelica.Blocks.Interfaces.SISO;
  parameter Real x0=0 "Start value of integrator state";
  parameter Real y0=0 "Start value of output";
  Modelica.Blocks.Interfaces.BooleanInput trigger
    annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
  Modelica.Blocks.Math.MultiProduct product(nu=2)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  TriggeredMean triggeredMean(
    final yGreaterOrEqualZero=true,
    final x0=x0,
    final y0=y0*y0) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Math.Sqrt sqrt1
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
equation
  connect(product.y, triggeredMean.u)
    annotation (Line(points={{-38.3,0},{-12,0}}, color={0,0,127}));
  connect(triggeredMean.y, sqrt1.u)
    annotation (Line(points={{11,0},{38,0}}, color={0,0,127}));
  connect(sqrt1.y, y) annotation (Line(
      points={{61,0},{110,0}}, color={0,0,127}));
  connect(u, product.u[1]) annotation (Line(
      points={{-120,0},{-80,0},{-80,3.5},{-60,3.5}}, color={0,0,127}));
  connect(u, product.u[2]) annotation (Line(
      points={{-120,0},{-80,0},{-80,-3.5},{-60,-3.5}}, color={0,0,127}));
  connect(trigger, triggeredMean.trigger)
    annotation (Line(points={{0,-120},{0,-12}}, color={255,0,255}));
  annotation (Documentation(info="<html>
<p>
This block calculates the root mean square of the input signal u over the period the period dtermined by the edges of the trigger:, using the
<a href=\"modelica://BLDC.CommonBlocks.TriggeredMean\">triggered mean block</a>.
</p>
<p>
Note: The output is updated at each trigger instance.
</p>
</html>"),
         Icon(graphics={Text(
          extent={{-80,60},{80,20}},
          textString="RMS")}));
end TriggeredRootMeanSquare;
