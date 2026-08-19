macro_rules! opcode_enum {
    ($name:ident { $($variant:ident = $value:expr),+ $(,)? }) => {
        #[derive(Clone, Copy, Debug, Eq, PartialEq)]
        #[repr(u16)]
        pub enum $name {
            $($variant = $value),+
        }

        impl TryFrom<u16> for $name {
            type Error = u16;

            fn try_from(value: u16) -> Result<Self, Self::Error> {
                match value {
                    $($value => Ok(Self::$variant),)+
                    _ => Err(value),
                }
            }
        }

        impl From<$name> for u16 {
            fn from(value: $name) -> Self {
                value as u16
            }
        }
    };
}

opcode_enum!(RequestOpcode {
    GetStatus = 0x0001,
    Grip = 0x0010,
    StartScan = 0x0011,
    Solve = 0x0012,
    Execute = 0x0013,
    ScanSolveExecute = 0x0014,
    Open = 0x0015,
    RecoverToOpen = 0x0016,
    ExecuteMoves = 0x0017,
    Abort = 0x00ff,
});

opcode_enum!(ResponseOpcode {
    CommandAccepted = 0x1000,
    CommandRejected = 0x1001,
    StatusSnapshot = 0x1002,
});

opcode_enum!(EventOpcode {
    RobotStateChanged = 0x2000,
    StandStateChanged = 0x2001,
    FaceScanned = 0x2002,
    PlanChanged = 0x2003,
    ActionStarted = 0x2004,
    ActionCompleted = 0x2005,
    OperationCompleted = 0x2006,
    Aborted = 0x2007,
    CubeSessionChanged = 0x2008,
    OperationFailed = 0x2009,
    Fault = 0x20ff,
});

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn abort_has_stable_priority_opcode() {
        assert_eq!(u16::from(RequestOpcode::Abort), 0x00ff);
        assert_eq!(RequestOpcode::try_from(0x00ff), Ok(RequestOpcode::Abort));
    }

    #[test]
    fn opcode_families_do_not_overlap() {
        assert!(u16::from(RequestOpcode::Abort) < 0x1000);
        assert!((0x1000..0x2000).contains(&u16::from(ResponseOpcode::StatusSnapshot)));
        assert!(u16::from(EventOpcode::RobotStateChanged) >= 0x2000);
    }
}
