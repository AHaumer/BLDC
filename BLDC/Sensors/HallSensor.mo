within BLDC.Sensors;
model HallSensor "Hall sensor"
  extends Modelica.Mechanics.Rotational.Interfaces.PartialOneFlangeAndSupport;
  import Modelica.Units.SI;
  import Modelica.Constants.pi;
  parameter Integer p(final min=1, start=2) "Number of pole pairs";
  parameter Integer m(min=3) = 3 "Number of stator phases";
  parameter SI.Angle orientation[m]=
    Modelica.Electrical.Polyphase.Functions.symmetricOrientation(m) "Orientation of phases";
  parameter SI.Angle phi0=0 "Initial mechanical angle (zero position)";
  Modelica.Blocks.Interfaces.RealOutput y(final unit="rad", displayUnit="deg")
    "Electrical angle"
    annotation (Placement(transformation(extent={{-100,50},{-120,70}})));
  Modelica.Blocks.Interfaces.BooleanOutput yC[m] "Commutation signals"
    annotation (Placement(transformation(extent={{-100,-10},{-120,10}})));
  Modelica.Mechanics.Rotational.Sensors.RelAngleSensor relAngleSensor
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={80,0})));
  Modelica.Blocks.Math.WrapAngle wrapAngle
    annotation (Placement(transformation(extent={{20,-10},{0,10}})));
  Modelica.Blocks.Math.Add add(k1=p, k2=p)
    annotation (Placement(transformation(extent={{50,-10},{30,10}})));
  Modelica.Blocks.Sources.Constant const(k=phi0)
    annotation (Placement(transformation(extent={{10,10},{-10,-10}},
        rotation=270,
        origin={60,-30})));
  Modelica.Blocks.Routing.Replicator replicator(nout=m)
    annotation (Placement(transformation(extent={{-20,-10},{-40,10}})));
  CommonBlocks.IntervalTest intervalTest[m](
    lowerLimit=lowerLimit,
    upperLimit=upperLimit,
    InsideInterval=InsideInterval)
    annotation (Placement(transformation(extent={{-50,-10},{-70,10}})));
protected
  parameter SI.Angle loLim[m]={Modelica.Math.wrapAngle(orientation[k] - pi/2, wrapAngle.positiveRange) for k in 1:m};
  parameter SI.Angle upLim[m]={Modelica.Math.wrapAngle(orientation[k] + pi/2, wrapAngle.positiveRange) for k in 1:m};
  parameter SI.Angle lowerLimit[m]={if loLim[k]<upLim[k] then loLim[k] else upLim[k] for k in 1:m};
  parameter SI.Angle upperLimit[m]={if loLim[k]<upLim[k] then upLim[k] else loLim[k] for k in 1:m};
  parameter Boolean InsideInterval[m]={loLim[k]<upLim[k] for k in 1:m};
equation
  connect(relAngleSensor.flange_a, internalSupport)
    annotation (Line(points={{80,-10},{80,-80},{0,-80}}, color={0,0,0}));
  connect(relAngleSensor.flange_b, flange)
    annotation (Line(points={{80,10},{90,10},{90,0},{100,0}}, color={0,0,0}));
  connect(wrapAngle.y, y)
    annotation (Line(points={{-1,0},{-10,0},{-10,60},{-110,60}},
                                                color={0,0,127}));
  connect(add.u2, const.y)
    annotation (Line(points={{52,-6},{60,-6},{60,-19}}, color={0,0,127}));
  connect(relAngleSensor.phi_rel, add.u1)
    annotation (Line(points={{69,0},{60,0},{60,6},{52,6}}, color={0,0,127}));
  connect(add.y, wrapAngle.u)
    annotation (Line(points={{29,0},{22,0}}, color={0,0,127}));
  connect(wrapAngle.y, replicator.u)
    annotation (Line(points={{-1,0},{-18,0}}, color={0,0,127}));
  connect(replicator.y, intervalTest.u)
    annotation (Line(points={{-41,0},{-48,0}}, color={0,0,127}));
  connect(intervalTest.y, yC)
    annotation (Line(points={{-71,0},{-110,0}}, color={255,0,255}));
   annotation (
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={95,95,95},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(points={{0,0},{0,70}},  color={95,95,95}),
        Line(points={{0,-70},{0,0}}, color={95,95,95}),
        Line(points={{-36,60},{-100,60}},
                                        color={95,95,95}),
        Line(points={{100,0},{70,0}}, color={95,95,95}),
        Text(
          extent={{-150,120},{150,80}},
          textColor={0,0,255},
          fillColor={255,255,255},
          textString="%name"),
        Line(points={{0,-30},{-0.545517,38.9449}},
                                     color={95,95,95},
          origin={-26,15},
          rotation=60),
        Line(points={{0,-30},{-0.545517,38.9449}},
                                     color={95,95,95},
          origin={34,-19},
          rotation=60),
        Line(points={{0,-30},{0.545517,38.9449}},
                                     color={95,95,95},
          origin={26,15},
          rotation=-60),
        Line(points={{0,-30},{0.545517,38.9449}},
                                     color={95,95,95},
          origin={-34,-19},
          rotation=-60),
        Ellipse(extent={{-20,20},{20,-20}}, lineColor={95,95,95},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-30,-10},{30,-70}},
          textColor={64,64,64},
          textString="rad"),
        Line(points={{-100,0},{-70,0}},   color={255,0,255})}),
    Documentation(info="<html>
<p>
Simple model of a hall sensor, i.e. measuring the angle of the flange (w.r.t. the optional support), multiplying by the number of pole pairs p to obtain the electrical angle,
and adding a correction term i.e. the initial angle of the flange phi0.
</p>
<p>
Additionally, the Boolean commutation signals for <code>m</code> phases are generated, 
i.e. an ouput <code>yC[k] = true</code> when the angle is within [-pi/2, +pi/2] of the orientation of phase k.
</p>
</html>"));
end HallSensor;
