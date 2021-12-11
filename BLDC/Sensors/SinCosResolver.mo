within BLDC.Sensors;
model SinCosResolver "Sin/Cos-Resolver"
  extends Modelica.Mechanics.Rotational.Interfaces.PartialOneFlangeAndSupport;
  import Modelica.Constants.pi;
  parameter Integer p(final min=1, start=2) "Number of pole pairs";
  parameter Real offset=1.5 "Offset of output";
  parameter Real amplitude=1 "Amplitude of output";
  parameter Modelica.Units.SI.Angle phi0=0 "Initial mechanical angle (zero position)";
  Modelica.Mechanics.Rotational.Sensors.RelAngleSensor relAngleSensor
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=270,
        origin={80,0})));
  Modelica.Blocks.Math.Add add(k1=p, k2=p)
    annotation (Placement(transformation(extent={{50,-10},{30,10}})));
  Modelica.Blocks.Sources.Constant const(k=phi0)
    annotation (Placement(transformation(extent={{10,10},{-10,-10}},
        rotation=270,
        origin={60,-30})));
  Modelica.Blocks.Math.Cos cos1
    annotation (Placement(transformation(extent={{10,-50},{-10,-30}})));
  Modelica.Blocks.Math.Sin sin1
    annotation (Placement(transformation(extent={{10,30},{-10,50}})));
  Modelica.Blocks.Interfaces.RealOutput y[4] "Resolver signals"
    annotation (Placement(transformation(extent={{-100,-10},{-120,10}}),
        iconTransformation(extent={{-100,-10},{-120,10}})));
  Modelica.Blocks.Math.Add add1(k1=+amplitude)
    annotation (Placement(transformation(extent={{-40,-50},{-60,-70}})));
  Modelica.Blocks.Math.Add addOffset2(k1=-amplitude)
    annotation (Placement(transformation(extent={{-40,-30},{-60,-10}})));
  Modelica.Blocks.Math.Add addOffset3(k1=amplitude)
    annotation (Placement(transformation(extent={{-40,30},{-60,10}})));
  Modelica.Blocks.Math.Add addOffset4(k1=-amplitude)
    annotation (Placement(transformation(extent={{-40,50},{-60,70}})));
  Modelica.Blocks.Sources.Constant constOffset(k=offset)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}})));
equation
  connect(add.u1, relAngleSensor.phi_rel)
    annotation (Line(points={{52,6},{60,6},{60,0},{69,0}}, color={0,0,127}));
  connect(const.y, add.u2)
    annotation (Line(points={{60,-19},{60,-6},{52,-6}}, color={0,0,127}));
  connect(add.y, sin1.u)
    annotation (Line(points={{29,0},{20,0},{20,40},{12,40}},color={0,0,127}));
  connect(add.y, cos1.u)
    annotation (Line(points={{29,0},{20,0},{20,-40},{12,-40}},color={0,0,127}));
  connect(addOffset4.y, y[4]) annotation (Line(points={{-61,60},{-80,60},{
          -80,7.5},{-110,7.5}},
                       color={0,0,127}));
  connect(addOffset3.y, y[3]) annotation (Line(points={{-61,20},{-70,20},{
          -70,2.5},{-110,2.5}},
                       color={0,0,127}));
  connect(addOffset2.y, y[2]) annotation (Line(points={{-61,-20},{-70,-20},
          {-70,-2.5},{-110,-2.5}},
                        color={0,0,127}));
  connect(add1.y, y[1]) annotation (Line(points={{-61,-60},{-80,-60},{-80,
          -7.5},{-110,-7.5}},
                        color={0,0,127}));
  connect(internalSupport, relAngleSensor.flange_a)
    annotation (Line(points={{0,-80},{80,-80},{80,-10}}, color={0,0,0}));
  connect(sin1.y, addOffset4.u1) annotation (Line(points={{-11,40},{-20,40},
          {-20,66},{-38,66}}, color={0,0,127}));
  connect(sin1.y, addOffset3.u1) annotation (Line(points={{-11,40},{-20,40},
          {-20,14},{-38,14}}, color={0,0,127}));
  connect(cos1.y, addOffset2.u1) annotation (Line(points={{-11,-40},{-20,
          -40},{-20,-14},{-38,-14}}, color={0,0,127}));
  connect(cos1.y, add1.u1) annotation (Line(points={{-11,-40},{-20,-40},{
          -20,-66},{-38,-66}}, color={0,0,127}));
  connect(constOffset.y, addOffset4.u2) annotation (Line(points={{-11,0},{
          -30,0},{-30,54},{-38,54}}, color={0,0,127}));
  connect(constOffset.y, addOffset3.u2) annotation (Line(points={{-11,0},{
          -30,0},{-30,26},{-38,26}}, color={0,0,127}));
  connect(constOffset.y, addOffset2.u2) annotation (Line(points={{-11,0},{
          -30,0},{-30,-26},{-38,-26}}, color={0,0,127}));
  connect(constOffset.y, add1.u2) annotation (Line(points={{-11,0},{-30,0},
          {-30,-54},{-38,-54}}, color={0,0,127}));
  connect(relAngleSensor.flange_b, flange) annotation (Line(points={{80,10},
          {90,10},{90,0},{100,0}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(extent={{-80,80},{80,-80}}, lineColor={95,95,95},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
    Text(textColor={0,0,255},
      extent={{-120,-20},{120,20}},
      textString="%name",
          origin={0,120},
          rotation=180),
        Line(points={{-80,0},{-100,0}},color={0,0,0}),
        Line(points={{80,0},{100,0}}, color={0,0,0}),
        Ellipse(extent={{-20,20},{20,-20}}, lineColor={95,95,95},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-80,80},{-74.4,78.1},{-68.7,72.3},{-63.1,63},{-56.7,48.7},{-48.6,
              26.6},{-29.3,-32.5},{-22.1,-51.7},{-15.7,-65.3},{-10.1,-73.8},{-4.42,
              -78.8},{1.21,-79.9},{6.83,-77.1},{12.5,-70.6},{18.1,-60.6},{24.5,-45.7},
              {32.6,-23},{50.3,31.3},{57.5,50.7},{63.9,64.6},{69.5,73.4},{75.2,78.6},
              {80,80}},
          smooth=Smooth.Bezier),
        Line(
          points={{-80,0},{-68.7,34.2},{-61.5,53.1},{-55.1,66.4},{-49.4,74.6},{-43.8,
              79.1},{-38.2,79.8},{-32.6,76.6},{-26.9,69.7},{-21.3,59.4},{-14.9,44.1},
              {-6.83,21.2},{10.1,-30.8},{17.3,-50.2},{23.7,-64.2},{29.3,-73.1},{35,
              -78.4},{40.6,-80},{46.2,-77.6},{51.9,-71.5},{57.5,-61.9},{63.9,-47.2},
              {72,-24.8},{80,0}},
          smooth=Smooth.Bezier)}),                               Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>
This is a model of sin/cos-encoder.
It senses the angle of the flange <code>phi</code> (w.r.t. the optional support), providing 4 analog signals:
</p>
<ul>
<li><code>y[1] = offset + amplitude*cos(p*(phi - phi0))</code></li>
<li><code>y[2] = offset - amplitude*cos(p*(phi - phi0))</code></li>
<li><code>y[3] = offset + amplitude*sin(p*(phi - phi0))</code></li>
<li><code>y[4] = offset - amplitude*sin(p*(phi - phi0))</code></li>
</ul>
<p>
It is used to measure angular velocity and rotor angle of electric drives. 
The offset of the output is used to sense whether the sensor is working or defect. 
It can easily be removed by subtracting <code>y[1] - y[2]</code> and <code>y[3] - y[4]</code>.
</p>
<h4>Note:</h4>
<p>
Usually the number of pole pairs <code>p</code> matches the number of pole pairs of the permanent magnet synchronous machine. 
The sensor directly provides the electrical rotor angle, i.e. <code>p*phi<sub>Mechancial</sub></code>, and the electrial angular velocity, i.e. <code>p*w<sub>Mechancial</sub></code>, 
if block <a href=\"modelica://BLDC.Utilities.SinCosEvaluation\">SinCosEvaluation</a> is used.
</p>
</html>"));
end SinCosResolver;
