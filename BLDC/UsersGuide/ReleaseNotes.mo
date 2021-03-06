within BLDC.UsersGuide;
class ReleaseNotes "Release Notes"
  extends Modelica.Icons.ReleaseNotes;
  annotation (Documentation(info="<html>

<h4>v2.0.0 2021-12-20</h4>
<p>enhanced test examples to evaluate parameters of a permanent magnet synchronous machine</p>

<h4>v1.9.0 2021-12-18</h4>
<p>adapted HallTimeSpan to translate with OM</p>

<h4>v1.8.0 2021-12-17</h4>
<ul>
<li>implemented an encode evaluation based on a combination of time measurement and pulse count</li>
<li>unified handling of direction of rotation and simlified when-conditions in encode evaluations</li>
</ul>

<h4>v1.7.0 2021-12-16</h4>
<ul>
<li>extended ParameterRecords.SmpmData to ease implementation of other machine parameter sets</li>
<li>suggestion: remove final from MSL.InductionmachineData(m, statorCoreParameters(m)) to enable handling of other number of phases</li>
<li>implemented example SMPM_NominalOperation to validate nominal operation</li>
</ul>

<h4>v1.6.0 2021-12-15</h4>
<ul>
<li>replaced getEventIndex by firstTrueIndex</li>
<li>vectorized loops for sake of performance</li>
</ul>

<h4>v1.5.0 2021-12-14</h4>
<ul>
<li>double-checked the electronic commutator</li>
<li>replaced utility function nextIndex by (more flexible) addIndex</li>
<li>added again HallTimeSpan to DemoEncoder for testing</li>
</ul>

<h4>v1.4.0 2021-12-13</h4>
<ul>
<li>Implemented some blocks to investigate current measurement</li>
<li>Started simplification and clarification of the electronic commutator</li>
<li>Revised the structure of packages</li>
</ul>

<h4>v1.3.3 2021-12-12</h4>
<ul>
<li>Simplified the electronic commutator</li>
<li>(Temporarily) removed HallTimeSpan from DemoEncoder</li>
</ul>

<h4>v1.3.2 2021-12-12</h4>
<p>(Temporarily) included a copy of the SignalPWM from master branch of ModelicaStandardLibrary (availability of triangular carrier).</p>

<h4>v1.3.1 2021-12-12</h4>
<p>Improved electronic commutator working properly for odd number of phases, even number of phases needs further investigation.<br>
   Note that number of phases = power of 2 is not handled by the FundamentalWave machine models.</p>

<h4>v1.3.0 2021-12-12</h4>
<ul>
<li>implemented direction of rotation from hall signals</code></li>
<li>investigate BLDC with number of phases <code>m&gt;3</code></li>
</ul>

<h4>v1.2.0 2021-12-11</h4>
<ul>
<li>improved CommonBlocks.IntervalTest</li>
<li>improved Utilities.ElectronicCommutator3phase</li>
<li>implemented polyphase Utilities.ElectronicCommutator <code>m&gt;3</code></li>
</ul>

<h4>v1.1.0 2021-12-11</h4>
<ul>
<li>corrected angles</li>
<li>under investigation:<br>
 HallTimeSpan: direction of rotation from hall signals<br>
 ElectronicCommutator for number of phases <code>m&gt;3</code></li>
</ul>

<h4>v1.0.0 2021-12-10</h4>
<p>First fully working version</p>

</html>"));
end ReleaseNotes;
