package frc.robot.utils.hardware;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class NeoBuilder {
    private Neo motor;
    private SparkMaxConfig config;

    private NeoBuilder(int id) {
        motor = new Neo(id);
        config = new SparkMaxConfig();
    }

    public NeoBuilder asFollower(SparkFlex motor, boolean inverted) {
        this.config.follow(motor, inverted);

        return this;
    }

    public NeoBuilder withVoltageCompensation(int nominalVoltage) {
        config.voltageCompensation(nominalVoltage);

        return this;
    }

    public NeoBuilder withCurrentLimit(int currentLimit) {
        config.smartCurrentLimit(currentLimit);

        return this;
    }

    public NeoBuilder withIdleMode(IdleMode mode) {
        config.idleMode(mode);

        return this;
    }

    public NeoBuilder withInversion(boolean inverted) {
        config.inverted(inverted);

        return this;
    }

    public NeoBuilder withPIDParams(double p, double i, double d) {
        config.closedLoop.pid(p, i, d);

        return this;
    }

    public NeoBuilder withPIDFParams(double p, double i, double d, double f) {
        config.closedLoop.pidf(p, i, d, f);

        return this;
    }

    public NeoBuilder withPIDClamping(double min, double max) {
        config.closedLoop.minOutput(min);
        config.closedLoop.maxOutput(max);

        return this;
    }

    public NeoBuilder withMaxIAccum(double max) {
        config.closedLoop.iMaxAccum(max);
        
        return this;
    }

    public NeoBuilder withPositionConversionFactor(double factor) {
        config.encoder.positionConversionFactor(factor);

        return this;
    }

    public NeoBuilder withPosition(double position) {
        motor.getEncoder().setPosition(position);

        return this;
    }

    public NeoBuilder withVelocityConversionFactor(double factor) {
        config.encoder.velocityConversionFactor(factor);

        return this;
    }

    public NeoBuilder withForwardSoftlimit(double limit) {
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit(limit);

        return this;
    }

    public NeoBuilder withReverseSoftLimit(double limit) {
        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(limit);

        return this;
    }

    public NeoBuilder withSoftLimits(double forwardLimit, double reverseLimit) {
        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(reverseLimit);

        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit(forwardLimit);

        return this;
    }

    public NeoBuilder withOutputRange(double min, double max) {
        config.closedLoop.outputRange(min, max);

        return this;
    }

    public NeoBuilder withClosedLoopRampRate(double rate) {
        config.closedLoopRampRate(rate);

        return this;
    }

    public NeoBuilder withOpenLoopRampRate(double rate) {
        config.openLoopRampRate(rate);

        return this;
    }

    public NeoBuilder withPIDPositionWrapping(double min, double max) {
        config.closedLoop.positionWrappingEnabled(true);
        config.closedLoop.positionWrappingInputRange(min, max);

        return this;
    }

    public NeoBuilder withAbsoluteEncoder() {
        config.absoluteEncoder.setSparkMaxDataPortConfig();

        return this;
    }

    public Neo build() {
        motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return motor;
    }

    public static NeoBuilder create(int id) {
        return new NeoBuilder(id);
    }
}