package org.firstinspires.ftc.teamcode.Systems.Token;

import java.util.function.Function;

import kotlin.jvm.functions.Function0;

@FunctionalInterface
public interface Tokenable {
    Function0<Boolean> getToken();


}
