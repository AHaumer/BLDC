within BLDC.BaseBlocks;
partial block BaseElectronicCommutator "partial electronic commutator"
  extends Modelica.Blocks.Icons.BooleanBlock;
  parameter Integer m(min=3) = 3 "Number of stator phases";
  parameter Modelica.Units.SI.Angle orientation[m]=
    Modelica.Electrical.Polyphase.Functions.symmetricOrientation(m) "Orientation of phases";
  parameter Boolean useConstantPWM=true "Otherwise input" annotation(Evaluate=true);
  parameter Boolean ConstantPWM=true "PWM resp. direction" annotation(Dialog(enable=useCOnstantPWM));
  Modelica.Blocks.Interfaces.BooleanInput pwm if not useConstantPWM
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.BooleanInput uC[m] "Commutation signals"
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
  Modelica.Blocks.Interfaces.BooleanOutput fire_p[m]
    "Fire signals of positive potential transistors"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Modelica.Blocks.Interfaces.BooleanOutput fire_n[m]
    "Fire signals of negative potential transistors"
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
protected
  Modelica.Blocks.Interfaces.BooleanInput internalPWM annotation (Placement(
        transformation(extent={{-94,56},{-86,64}}), iconTransformation(extent={{-94,
            56},{-86,64}})));
equation
  if useConstantPWM then
    internalPWM=ConstantPWM;
  else
    connect(internalPWM, pwm);
  end if;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{-60,30},{-60,50},{0,50},{0,30},{60,30},{60,50}}, color={255,
              0,255}),
        Line(points={{-60,-10},{-20,-10},{-20,10},{40,10},{40,-10},{60,-10}},
            color={255,0,255}),
        Line(points={{-60,-30},{-40,-30},{-40,-50},{20,-50},{20,-30},{60,-30}},
            color={255,0,255}),
        Text(
          extent={{-100,-70},{100,-90}},
          textColor={0,0,0},
          textString="m=%m")}),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>
Base block for 3 phase and polyphase electronic commutator.
</p>
</html>"));
end BaseElectronicCommutator;
