within BLDC.ParameterRecords;
record Smpm1FT7102
  "Parameters for 1FT7102"
  import Modelica.Units.SI;
  import Modelica.Constants.pi;
  import Modelica.Units.Conversions.from_degC;
  import Modelica.Electrical.Machines.Thermal.Constants.alpha20Copper;
  extends SmpmData(
    MachineType="1FT7102",
    tauNominal=28.09,
    wNominal=2*pi*fsNominal/p,
    IsNominal=8,
    VsNominal=205.66,
    TsNominal=from_degC(95),
    TrNominal=from_degC(95),
    Jr=9.14e-3,
    p=5,
    fsNominal=125,
    VsOpenCircuit=212.3/sqrt(3)*1.5,
    Rs=m/3*0.60,
    TsRef=from_degC(20),
    alpha20s=alpha20Copper,
    Lssigma=m/3*0.1*12.5e-3,
    Lmd=m/3*0.9*12.5e-3,
    Lmq=Lmd,
    useDamperCage=false,
    frictionParameters(
      PRef=0,
      wRef=wNominal,
      power_w=2),
    statorCoreParameters(
      PRef=0,
      VRef=VsOpenCircuit,
      wRef=wNominal),
    strayLoadParameters(
      PRef=0,
      IRef=IsNominal,
      wRef=wNominal,
      power_w=1),
    permanentMagnetLossParameters(
      PRef=0,
      c=0,
      IRef=IsNominal,
      power_I=2,
      wRef=wNominal,
      power_w=1));
  annotation (
    defaultComponentName="smpmData",
    defaultComponentPrefixes="parameter",
    Documentation(info="<html>
<p>Parameters of a synchronous machine with permanent magnet 1FT7102.</p>
</html>"),
    Icon(graphics={Text(
          extent={{-100,0},{100,-40}},
          textColor={28,108,200},
          textString="%MachineType")}));
end Smpm1FT7102;
