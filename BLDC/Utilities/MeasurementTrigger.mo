within BLDC.Utilities;
block MeasurementTrigger "Detect measurement trigger from rising and falling edges"
  extends Modelica.Blocks.Interfaces.partialBooleanSO;
  parameter Integer m(min=3) = 3 "Number of stator phases";
  Modelica.Blocks.Interfaces.BooleanInput u[m]
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Logical.Edge risingEdge[m]
    annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
  Modelica.Blocks.Logical.FallingEdge fallingEdge[m]
    annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
  CommonBlocks.MultiOr multiOr(nin=2*m)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
equation
  connect(u, risingEdge.u) annotation (Line(points={{-120,0},{-60,0},{-60,20},{
          -42,20}}, color={255,0,255}));
  connect(u, fallingEdge.u) annotation (Line(points={{-120,0},{-60,0},{-60,-20},
          {-42,-20}}, color={255,0,255}));
  connect(risingEdge.y, multiOr.u[1:m])
    annotation (Line(points={{-19,20},{0,20},{0,0},{18,0}}, color={255,0,255}));
  connect(fallingEdge.y, multiOr.u[m + 1:2*m]) annotation (Line(points={{-19,-20},
          {0,-20},{0,0},{18,0}}, color={255,0,255}));
  connect(multiOr.y, y)
    annotation (Line(points={{41,0},{110,0}}, color={255,0,255}));
end MeasurementTrigger;
