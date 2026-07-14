using DigitalTwin.Kinematics;
using NUnit.Framework;
using UnityEngine;

public sealed class RobotKinematicsTests
{
    [Test]
    public void ForwardKinematicsStraightArmSumsLinkLengths()
    {
        RobotKinematicParameters parameters = CreateParameters();
        ForwardKinematicsResult fk = RobotKinematics.ForwardKinematics(parameters, new[] { 0f, 0f, 0f });

        Assert.That(fk.EndEffectorPosition.x, Is.EqualTo(0.60f).Within(1e-5f));
        Assert.That(fk.EndEffectorPosition.y, Is.EqualTo(0f).Within(1e-5f));
        Assert.That(fk.LinkSegments.Length, Is.EqualTo(3));
    }

    [Test]
    public void PointToSegmentDistanceClampsToFiniteSegment()
    {
        float distance = RobotKinematics.PointToSegmentDistance(
            new Vector3(0.5f, 0.3f, 0f),
            Vector3.zero,
            Vector3.right,
            out Vector3 closest);

        Assert.That(distance, Is.EqualTo(0.3f).Within(1e-5f));
        Assert.That(closest.x, Is.EqualTo(0.5f).Within(1e-5f));
    }

    [Test]
    public void SafetyDecisionUsesStopReleaseHysteresis()
    {
        SafetyDecision holdStop = RobotKinematics.DecideSafety(
            true,
            false,
            false,
            0.30f,
            0.45f,
            0.25f,
            0.35f,
            SafetyDecision.Stop);

        Assert.That(holdStop, Is.EqualTo(SafetyDecision.Stop));

        SafetyDecision released = RobotKinematics.DecideSafety(
            true,
            false,
            false,
            0.40f,
            0.45f,
            0.25f,
            0.35f,
            SafetyDecision.Stop);

        Assert.That(released, Is.EqualTo(SafetyDecision.Warning));
    }

    [Test]
    public void StaleDataIsUnsafe()
    {
        SafetyDecision decision = RobotKinematics.DecideSafety(
            false,
            false,
            false,
            10f,
            0.45f,
            0.25f,
            0.35f,
            SafetyDecision.Safe);

        Assert.That(decision, Is.EqualTo(SafetyDecision.Stop));
    }

    private static RobotKinematicParameters CreateParameters()
    {
        RobotKinematicParameters parameters = ScriptableObject.CreateInstance<RobotKinematicParameters>();
        parameters.basePositionMeters = Vector3.zero;
        parameters.horizontalAxis = Vector3.right;
        parameters.verticalAxis = Vector3.up;
        parameters.links = new[]
        {
            new RobotLinkParameter("L1", 0.25f, 0f),
            new RobotLinkParameter("L2", 0.20f, 0f),
            new RobotLinkParameter("L3", 0.15f, 0f)
        };
        return parameters;
    }
}
