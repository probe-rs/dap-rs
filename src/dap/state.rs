use crate::{jtag, swd, swj};

/// State machine of the Dap handler
pub enum State<DEPS, SWD, JTAG> {
    /// State that _should_ never happen.
    ///
    /// Assumption: No one tries to "somehow" catch the `From::from` panic for
    /// None/SWD/JTAG.
    Invalid,
    /// None/uninitialized/direct mode
    ///
    /// Used to execute CMSIS DAP commands that rely on a direct pin
    /// manipulation
    None {
        mode_to_restore: DynState,
        deps: DEPS,
    },
    /// SWD mode
    Swd(SWD),
    /// JTAG mode
    Jtag(JTAG),
}

/// Plain enum describing the current state
///
/// Used to record the previous non-`None` mode in order to restore in
/// [`State::to_last_mode`]
pub enum DynState {
    None,
    Swd,
    Jtag,
}

impl<DEPS, SWD, JTAG> State<DEPS, SWD, JTAG>
where
    DEPS: swj::Dependencies<SWD, JTAG>,
    SWD: swd::Swd<DEPS>,
    JTAG: jtag::Jtag<DEPS>,
{
    /// Change the clock configuration
    pub fn set_clock(&mut self, max_frequency: u32) -> bool {
        match self {
            State::None { deps, .. } => deps.process_swj_clock(max_frequency),
            // TODO: I think this is wrong or at least barely applicable approach.
            // `Swd` if running eg. SPI won't be able to change it's clock on fly
            // Switch to `None` and back will be probably necessary and this is also
            // what should be done here?
            State::Swd(v) => v.set_clock(max_frequency),
            State::Jtag(v) => v.set_clock(max_frequency),
            State::Invalid => unreachable!(),
        }
    }
}

impl<DEPS, SWD, JTAG> State<DEPS, SWD, JTAG>
where
    DEPS: From<SWD> + From<JTAG>,
    SWD: From<DEPS>,
    JTAG: From<DEPS>,
{
    /// Construct an instance of [`State`] object
    pub fn new(deps: DEPS) -> Self {
        Self::None {
            deps,
            mode_to_restore: DynState::None,
        }
    }

    /// Force the state transition to `None`.
    ///
    /// Useful for commands that rely on the direct pin control.
    pub fn to_none(&mut self) {
        match self {
            State::None { .. } => {}
            state => state.replace_with(|s| match s {
                State::Swd(v) => State::None {
                    deps: v.into(),
                    mode_to_restore: DynState::Swd,
                },
                State::Jtag(v) => State::None {
                    deps: v.into(),
                    mode_to_restore: DynState::Jtag,
                },
                State::Invalid | State::None { .. } => unreachable!(),
            }),
        }
    }

    /// Force the state transition to the last non-`None` mode.
    ///
    /// Useful for commands that do the proper data transmition in SWD/JTAG mode
    pub fn to_last_mode(&mut self) {
        match self {
            State::Swd(_) | State::Jtag(_) => {}
            state @ State::None { .. } => state.replace_with(|s| match s {
                State::None {
                    mode_to_restore,
                    deps,
                } => match mode_to_restore {
                    DynState::None => State::None {
                        mode_to_restore,
                        deps,
                    },
                    DynState::Swd => State::Swd(deps.into()),
                    DynState::Jtag => State::Jtag(deps.into()),
                },
                State::Swd(_) | State::Jtag(_) | State::Invalid => unreachable!(),
            }),
            State::Invalid => unreachable!(),
        }
    }

    /// Force the state transition to SWD.
    ///
    /// Useful for commands that specify the transmission protocol to be SWD.
    pub fn to_swd(&mut self) {
        match self {
            State::Swd(_) => {}
            state => state.replace_with(|s| match s {
                State::None { deps, .. } => State::Swd(deps.into()),
                State::Jtag(v) => State::Swd(DEPS::from(v).into()),
                State::Swd(_) | State::Invalid => unreachable!(),
            }),
        }
    }

    /// Force the state transition to JTAG.
    ///
    /// Useful for commands that specify the transmission protocol to be JTAG.
    pub fn to_jtag(&mut self) {
        match self {
            State::Jtag(_) => {}
            state => state.replace_with(|s| match s {
                State::None { deps, .. } => State::Jtag(deps.into()),
                State::Swd(v) => State::Jtag(DEPS::from(v).into()),
                State::Jtag(_) | State::Invalid => unreachable!(),
            }),
        }
    }

    #[inline(always)]
    fn replace_with<F: FnOnce(Self) -> Self>(&mut self, f: F) {
        replace_with::replace_with(self, || State::Invalid, f);
    }
}
