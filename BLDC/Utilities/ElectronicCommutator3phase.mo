within BLDC.Utilities;
block ElectronicCommutator3phase "Electronic commutator for 3 phases"
  extends Modelica.Blocks.Icons.BooleanBlock;
  constant Integer m=3 "Number of phases";
  parameter Boolean useConstantPWM=true "Otherwise input" annotation(Evaluate=true);
  parameter Boolean ConstantPWM=true "PWM resp. direction" annotation(Dialog(enable=useConstantPWM));
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
  constant Integer I1[6]={2,3,2,1,1,3} "Indices of on states in sectors";
  constant Integer I0[6]={3,1,1,2,3,2} "Indices of off states in sectors";
  constant Integer Ix[6]={1,2,3,3,2,1} "Indices of open states in sectors";
  //Angle    0  60 120 180 240 300
  //Sector   1   2   3   4   5   6
  //Hall   100 110 010 011 001 101
  //Fire   x10 01x 0x1 x01 10x 1x0
  Integer c=sum({if uC[k] then integer(2^(k - 1)) else 0 for k in 1:m}) "Sector code";
  Modelica.Blocks.Interfaces.BooleanInput internalPWM annotation (Placement(
        transformation(extent={{-94,56},{-86,64}}), iconTransformation(extent={{-94,
            56},{-86,64}})));
equation
  if useConstantPWM then
    internalPWM=ConstantPWM;
  else
    connect(internalPWM, pwm);
  end if;
algorithm
  assert(c>=1 and c<=6, "Hall sensor sector error!");
  fire_p[I1[c]]:=internalPWM;
  fire_n[I1[c]]:=not internalPWM;
  fire_p[I0[c]]:=not internalPWM;
  fire_n[I0[c]]:=internalPWM;
  fire_p[Ix[c]]:=false;
  fire_n[Ix[c]]:=false;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{-60,30},{-60,50},{0,50},{0,30},{60,30},{60,50}}, color={255,
              0,255}),
        Line(points={{-60,-10},{-20,-10},{-20,10},{40,10},{40,-10},{60,-10}},
            color={255,0,255}),
        Line(points={{-60,-30},{-40,-30},{-40,-50},{20,-50},{20,-30},{60,-30}},
            color={255,0,255})}),                                Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>
According to the Hall sensor signals <code>uC[m=3]</code>, the inactive (open) leg is choossen 
as well as the two legs that apply the DC voltage to two terminals of the machine, 
and therefore let a current flow through the corresponding phases of the machine 
to obtain a current space phasor perpendicular to the rotor's magnetic field. 
Of course due to the block commutation the angle between rotor position and current space phasor 
varies in the range <code>[&#177;90&deg; - 30&deg;, &#177;90&deg; + 30&deg;]</code>.
</p>
<p>
Setting the input <code>pwm = true</code> chooses the position of the current space phasor as <code>+90&deg;</code>, whereas 
setting the input <code>pwm = false</code> results in a position of the current space phasor as <code>-90&deg;</code> w.r.t. the rotor position. 
Applying a pwm signal (as for a brushed DC machine) to the input <code>pwm</code> offers the possibility to set the mean voltage applied to the phases choosen according to the Hall signals.
</p>
</html>"));
end ElectronicCommutator3phase;
