package org.firstinspires.ftc.teamcode.Systems.Token;

import com.acmerobotics.roadrunner.Action;

public abstract class TokenAction implements Action {
    public abstract Tokenable getToken();

    public boolean checkToken(Tokenable token){
        return token.checkToken().invoke();
    }
}
