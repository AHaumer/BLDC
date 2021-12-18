within BLDC.Examples;
model DemoBLDCVoltages "Test example: Demonstrate BLDC voltages"
  extends Modelica.Icons.Example;
  import Modelica.Units.SI;
  import Modelica.Constants.pi;
  parameter Integer m=3 "Number of phases";
  parameter Integer p(final min=1)=2 "Number of pole pairs";
  parameter SI.Voltage VDC=100*Modelica.Electrical.Polyphase.Functions.factorY2DC(m) "Nominal DC voltage";
  parameter SI.Frequency fNominal=50 "Nominal frequqncy";
  parameter SI.AngularVelocity wNominal(displayUnit="rpm")=2*pi*fNominal/p "Nominal speed";
  Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(
    w_fixed=0.1*wNominal,  phi(start=0, fixed=true))
    annotation (Placement(transformation(extent={{-90,10},{-70,30}})));
  BLDC.Sensors.HallSensor hallSensor(p=p, m=m) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-50,20})));
  Utilities.ElectronicCommutator electronicCommutator(m=m)
    annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
  Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level inverter(m=m)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,40})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
    annotation (Placement(transformation(extent={{10,70},{-10,90}})));
  Modelica.Electrical.Analog.Basic.Ground groundDC annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,70})));
  Modelica.Electrical.Polyphase.Sensors.VoltageSensor voltageSensor(m=m)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={0,10})));
  Modelica.Electrical.Polyphase.Basic.Star star(m=m) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-20})));
  Modelica.Electrical.Machines.SpacePhasors.Blocks.ToSpacePhasor toSpacePhasor(m=m)
    annotation (Placement(transformation(extent={{20,0},{40,20}})));
  Modelica.Electrical.Machines.SpacePhasors.Blocks.Rotator rotator
    annotation (Placement(transformation(extent={{50,0},{70,20}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=1e6) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-50})));
  Modelica.Electrical.Analog.Basic.Ground groundAC
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}})));
equation
  connect(constantVoltage.p, inverter.dc_p) annotation (Line(points={{10,80},
          {10,70},{6,70},{6,50}},color={0,0,255}));
  connect(electronicCommutator.fire_p, inverter.fire_p)
    annotation (Line(points={{-19,46},{-12,46}}, color={255,0,255}));
  connect(electronicCommutator.fire_n, inverter.fire_n)
    annotation (Line(points={{-19,34},{-12,34}}, color={255,0,255}));
  connect(hallSensor.yC, electronicCommutator.uC)
    annotation (Line(points={{-39,20},{-30,20},{-30,28}}, color={255,0,255}));
  connect(constantSpeed.flange, hallSensor.flange)
    annotation (Line(points={{-70,20},{-60,20}}, color={0,0,0}));
  connect(inverter.ac, voltageSensor.plug_p)
    annotation (Line(points={{-1.77636e-15,30},{-1.77636e-15,26},{0,26},{0,
          20}},                                color={0,0,255}));
  connect(voltageSensor.plug_n, star.plug_p)
    annotation (Line(points={{0,0},{0,-10}},     color={0,0,255}));
  connect(voltageSensor.v, toSpacePhasor.u)
    annotation (Line(points={{11,10},{18,10}},
                                             color={0,0,127}));
  connect(toSpacePhasor.y, rotator.u)
    annotation (Line(points={{41,10},{48,10}},
                                             color={0,0,127}));
  connect(hallSensor.y, rotator.angle) annotation (Line(points={{-39,14},{
          -30,14},{-30,-90},{60,-90},{60,-2}},
                                        color={0,0,127}));
  connect(constantVoltage.n, groundDC.p)
    annotation (Line(points={{-10,80},{-10,70}}, color={0,0,255}));
  connect(groundDC.p, inverter.dc_n)
    annotation (Line(points={{-10,70},{-6,70},{-6,50}}, color={0,0,255}));
  connect(star.pin_n, resistor.p)
    annotation (Line(points={{0,-30},{0,-40}},   color={0,0,255}));
  connect(resistor.n, groundAC.p)
    annotation (Line(points={{0,-60},{0,-70}}, color={0,0,255}));
  annotation (experiment(
      StopTime=1,
      Interval=1e-05,
      Tolerance=1e-06), Documentation(info="<html>
<p>
Demonstrates how phase voltages are built from a constant DC voltage and Hall sensor signals, spinning at constant speed. 
Plot the resulting voltage space vector <code>rotator.y[1] + j*rotator.y[2]</code>.
</p>
</html>"));
end DemoBLDCVoltages;
