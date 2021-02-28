use core::marker::PhantomData;
use cortex_m_log::modes::InterruptModer;
use cortex_m_log::printer::Printer;

type Tx = stm32h7xx_hal::serial::Tx<stm32h7xx_hal::stm32::USART3>;

pub struct UartPrinter<M: InterruptModer> {
    tx: Tx,
    _mod: PhantomData<M>,
}

impl<M: InterruptModer> UartPrinter<M> {
    pub fn new(tx: Tx) -> Self {
        Self {
            tx: tx,
            _mod: PhantomData,
        }
    }
}

impl<Mode: InterruptModer> Printer for UartPrinter<Mode> {
    type W = Tx;
    type M = Mode;

    #[inline]
    fn destination(&mut self) -> &mut Self::W {
        &mut self.tx
    }
}

// Note(unsafe): Assume Sync for UartPrinter so that InterruptOk can be used for the
// global logger, even if that could lead to corrupted output when log output is
// interrupted by another log message from an interrupt context.
unsafe impl<Mode: InterruptModer> Sync for UartPrinter<Mode> {}
