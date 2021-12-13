within BLDC.Sensors;
model IncrementalEncoder "Incremental encoder"
  extends Modelica.Mechanics.Rotational.Interfaces.PartialOneFlangeAndSupport;
  import Modelica.Constants.pi;
  parameter Integer pRev(final min=1, start=128) "Pulses per revolution";
  Modelica.Blocks.Interfaces.BooleanOutput y[3] "Encoder signals"
    annotation (Placement(transformation(extent={{-100,-10},{-120,10}})));
  Modelica.Mechanics.Rotational.Sensors.RelAngleSensor relAngleSensor
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={80,0})));
  Modelica.Blocks.Math.WrapAngle wrapAngle1(positiveRange=true)
    annotation (Placement(transformation(extent={{60,-10},{40,10}})));
  Modelica.Blocks.Math.Gain gain(k=pRev)
    annotation (Placement(transformation(extent={{20,-10},{0,10}})));
  ToMSL.IntervalTest intervalTest1(
    constantLowerLimit=0,
    constantUpperLimit=2*pi/pRev/4,
    InsideInterval=true)
    annotation (Placement(transformation(extent={{-60,-40},{-80,-20}})));
  ToMSL.IntervalTest intervalTest2(
    constantLowerLimit=pi/2,
    constantUpperLimit=3*pi/2,
    InsideInterval=false)
    annotation (Placement(transformation(extent={{-60,-10},{-80,10}})));
  ToMSL.IntervalTest intervalTest3(
    constantLowerLimit=0,
    constantUpperLimit=pi,
    InsideInterval=true)
    annotation (Placement(transformation(extent={{-60,20},{-80,40}})));
  Modelica.Blocks.Math.WrapAngle wrapAngle2(positiveRange=true)
    annotation (Placement(transformation(extent={{-10,-10},{-30,10}})));
equation
  connect(relAngleSensor.flange_b, flange)
    annotation (Line(points={{80,10},{100,10},{100,0}}, color={0,0,0}));
  connect(internalSupport, relAngleSensor.flange_a) annotation (Line(points={{0,-80},
          {0,-60},{80,-60},{80,-10}}, color={0,0,0}));
  connect(relAngleSensor.phi_rel, wrapAngle1.u)
    annotation (Line(points={{69,0},{62,0}}, color={0,0,127}));
  connect(wrapAngle1.y, gain.u)
    annotation (Line(points={{39,0},{22,0}}, color={0,0,127}));
  connect(intervalTest2.y, y[2])
    annotation (Line(points={{-81,0},{-110,0}}, color={255,0,255}));
  connect(intervalTest3.y, y[3]) annotation (Line(points={{-81,30},{-90,30},{-90,
          6.66667},{-110,6.66667}}, color={255,0,255}));
  connect(intervalTest1.y, y[1]) annotation (Line(points={{-81,-30},{-90,-30},{-90,
          -6.66667},{-110,-6.66667}}, color={255,0,255}));
  connect(wrapAngle1.y, intervalTest1.u) annotation (Line(points={{39,0},{30,
          0},{30,-30},{-58,-30}}, color={0,0,127}));
  connect(gain.y, wrapAngle2.u)
    annotation (Line(points={{-1,0},{-8,0}}, color={0,0,127}));
  connect(wrapAngle2.y, intervalTest2.u)
    annotation (Line(points={{-31,0},{-58,0}}, color={0,0,127}));
  connect(wrapAngle2.y, intervalTest3.u) annotation (Line(points={{-31,0},{
          -50,0},{-50,30},{-58,30}}, color={0,0,127}));
  annotation (Icon(graphics={
        Line(points={{-70,0},{-100,0}}, color={95,95,95}),
        Line(points={{100,0},{70,0}}, color={95,95,95}),
        Text(
          extent={{-150,120},{150,80}},
          textColor={0,0,255},
          fillColor={255,255,255},
          textString="%name"),
        Ellipse(extent={{-80,80},{80,-80}}, lineColor={95,95,95},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Ellipse(extent={{-20,20},{20,-20}}, lineColor={95,95,95},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,-40},{-60,-40},{-60,-20},{-20,-20},{-20,-40},{20,
              -40},{20,-20},{60,-20},{60,-40},{80,-40}},
                                        color={0,0,0}),
        Line(points={{80,20},{40,20},{40,40},{0,40},{0,20},{-40,20},{-40,40},
              {-80,40},{-80,20}},      color={0,0,0})}),
                                            Documentation(info="<html>
<p>
This is a model of an incremental encoder.  
It senses the angle of the flange <code>phi</code> (w.r.t. the optional support), providing 3 Boolean signals:
</p>
<ul>
<li>0-track <code>y[1]</code> = 1 pulse per revolution with a length of <code>2*pi/(4*pRev)</code></li>
<li>A-track <code>y[2]</code> = <code>pRev</code> pulses per revolution</li>
<li>B-track <code>y[3]</code> = <code>pRev</code> pulses per revolution, displaced by <code>2*pi/(4*pRev)</code></li>
</ul>
<p>
It is used to measure angular velocity of electric drives, using block <a href=\"modelica://BLDC.Utilities.EncoderTimeSpan\">EncoderTimeSpan</a> 
or <a href=\"modelica://BLDC.Utilities.EncoderPulseCount\">EncoderPulseCount</a>.
</p>
</html>"));
end IncrementalEncoder;
