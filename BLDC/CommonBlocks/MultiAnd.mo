within BLDC.CommonBlocks;
block MultiAnd "Logical and of Boolean vector elements"
  extends Modelica.Blocks.Interfaces.partialBooleanSO;
  parameter Integer nin=2 "Number of inputs";
  Modelica.Blocks.Interfaces.BooleanInput u[nin]
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
equation
  y=Modelica.Math.BooleanVectors.allTrue(u);
  annotation (Icon(graphics={ Text(
          extent={{-90,40},{90,-40}},
          textString="and")}), Documentation(info="<html>
<p>
The output is <strong>true</strong> if all elements of the input vector are <strong>true</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
end MultiAnd;
