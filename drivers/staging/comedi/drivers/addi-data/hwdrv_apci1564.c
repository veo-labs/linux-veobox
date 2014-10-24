/* Digital Input IRQ Function Selection */
#define APCI1564_DI_INT_OR				(0 << 1)
#define APCI1564_DI_INT_AND				(1 << 1)

/* Digital Input Interrupt Enable Disable. */
#define APCI1564_DI_INT_ENABLE				0x4
#define APCI1564_DI_INT_DISABLE				0xfffffffb

/* Digital Output Interrupt Enable Disable. */
#define APCI1564_DO_VCC_INT_ENABLE			0x1
#define APCI1564_DO_VCC_INT_DISABLE			0xfffffffe
#define APCI1564_DO_CC_INT_ENABLE			0x2
#define APCI1564_DO_CC_INT_DISABLE			0xfffffffd

/* TIMER COUNTER WATCHDOG DEFINES */
#define ADDIDATA_TIMER					0
#define ADDIDATA_COUNTER				1
#define ADDIDATA_WATCHDOG				2

/*
 * devpriv->amcc_iobase Register Map
 */
#define APCI1564_DI_REG					0x04
#define APCI1564_DI_INT_MODE1_REG			0x08
#define APCI1564_DI_INT_MODE2_REG			0x0c
#define APCI1564_DI_INT_STATUS_REG			0x10
#define APCI1564_DI_IRQ_REG				0x14
#define APCI1564_DO_REG					0x18
#define APCI1564_DO_INT_CTRL_REG			0x1c
#define APCI1564_DO_INT_STATUS_REG			0x20
#define APCI1564_DO_IRQ_REG				0x24
#define APCI1564_WDOG_REG				0x28
#define APCI1564_WDOG_RELOAD_REG			0x2c
#define APCI1564_WDOG_TIMEBASE_REG			0x30
#define APCI1564_WDOG_CTRL_REG				0x34
#define APCI1564_WDOG_STATUS_REG			0x38
#define APCI1564_WDOG_IRQ_REG				0x3c
#define APCI1564_WDOG_WARN_TIMEVAL_REG			0x40
#define APCI1564_WDOG_WARN_TIMEBASE_REG			0x44
#define APCI1564_TIMER_REG				0x48
#define APCI1564_TIMER_RELOAD_REG			0x4c
#define APCI1564_TIMER_TIMEBASE_REG			0x50
#define APCI1564_TIMER_CTRL_REG				0x54
#define APCI1564_TIMER_STATUS_REG			0x58
#define APCI1564_TIMER_IRQ_REG				0x5c
#define APCI1564_TIMER_WARN_TIMEVAL_REG			0x60
#define APCI1564_TIMER_WARN_TIMEBASE_REG		0x64

/*
 * dev->iobase Register Map
 */
#define APCI1564_COUNTER_REG(x)				(0x00 + ((x) * 0x20))
#define APCI1564_COUNTER_RELOAD_REG(x)			(0x04 + ((x) * 0x20))
#define APCI1564_COUNTER_TIMEBASE_REG(x)		(0x08 + ((x) * 0x20))
#define APCI1564_COUNTER_CTRL_REG(x)			(0x0c + ((x) * 0x20))
#define APCI1564_COUNTER_STATUS_REG(x)			(0x10 + ((x) * 0x20))
#define APCI1564_COUNTER_IRQ_REG(x)			(0x14 + ((x) * 0x20))
#define APCI1564_COUNTER_WARN_TIMEVAL_REG(x)		(0x18 + ((x) * 0x20))
#define APCI1564_COUNTER_WARN_TIMEBASE_REG(x)		(0x1c + ((x) * 0x20))

/*
 * Configures The Timer or Counter
 *
 * data[0] Configure as: 0 = Timer, 1 = Counter
 * data[1] 1 = Enable Interrupt, 0 = Disable Interrupt
 * data[2] Time Unit
 * data[3] Reload Value
 * data[4] Timer Mode
 * data[5] Timer Counter Watchdog Number
 * data[6] Counter Direction
 */
static int apci1564_timer_config(struct comedi_device *dev,
				 struct comedi_subdevice *s,
				 struct comedi_insn *insn,
				 unsigned int *data)
{
	struct apci1564_private *devpriv = dev->private;
	unsigned int ctrl;

	devpriv->tsk_current = current;
	if (data[0] == ADDIDATA_TIMER) {
		/* First Stop The Timer */
		ul_Command1 = inl(devpriv->amcc_iobase + APCI1564_TIMER_CTRL_REG);
		ul_Command1 = ul_Command1 & 0xFFFFF9FEUL;
		/* Stop The Timer */
		outl(ul_Command1, devpriv->amcc_iobase + APCI1564_TIMER_CTRL_REG);

		devpriv->timer_select_mode = ADDIDATA_TIMER;
		if (data[1] == 1) {
			/* Enable TIMER int & DISABLE ALL THE OTHER int SOURCES */
			outl(0x02, devpriv->amcc_iobase + APCI1564_TIMER_CTRL_REG);
			outl(0x0, devpriv->amcc_iobase + APCI1564_DI_IRQ_REG);
			outl(0x0, devpriv->amcc_iobase + APCI1564_DO_IRQ_REG);
			outl(0x0, devpriv->amcc_iobase + APCI1564_WDOG_IRQ_REG);
			outl(0x0, dev->iobase +
			    APCI1564_COUNTER_IRQ_REG(0));
			outl(0x0, dev->iobase +
			    APCI1564_COUNTER_IRQ_REG(1));
			outl(0x0, dev->iobase +
			    APCI1564_COUNTER_IRQ_REG(2));
			outl(0x0, dev->iobase +
			    APCI1564_COUNTER_IRQ_REG(3));
		} else {
			/* disable Timer interrupt */
			outl(0x0, devpriv->amcc_iobase + APCI1564_TIMER_CTRL_REG);
		}

	/* First Stop The Timer */
	ctrl = inl(devpriv->timer + ADDI_TCW_CTRL_REG);
	ctrl &= 0xfffff9fe;
	/* Stop The Timer */
	outl(ctrl, devpriv->timer + ADDI_TCW_CTRL_REG);

	if (data[1] == 1) {
		/* Enable timer int & disable all the other int sources */
		outl(0x02, devpriv->timer + ADDI_TCW_CTRL_REG);
		outl(0x0, dev->iobase + APCI1564_DI_IRQ_REG);
		outl(0x0, dev->iobase + APCI1564_DO_IRQ_REG);
		outl(0x0, dev->iobase + APCI1564_WDOG_IRQ_REG);
		if (devpriv->counters) {
			unsigned long iobase;

			iobase = devpriv->counters + ADDI_TCW_IRQ_REG;
			outl(0x0, iobase + APCI1564_COUNTER(0));
			outl(0x0, iobase + APCI1564_COUNTER(1));
			outl(0x0, iobase + APCI1564_COUNTER(2));
		}
	} else {
		/* disable Timer interrupt */
		outl(0x0, devpriv->timer + ADDI_TCW_CTRL_REG);
	}

	/* Loading Timebase */
	outl(data[2], devpriv->timer + ADDI_TCW_TIMEBASE_REG);

	/* Loading the Reload value */
	outl(data[3], devpriv->timer + ADDI_TCW_RELOAD_REG);

	ctrl = inl(devpriv->timer + ADDI_TCW_CTRL_REG);
	ctrl &= 0xfff719e2;
	ctrl |= (2 << 13) | 0x10;
	/* mode 2 */
	outl(ctrl, devpriv->timer + ADDI_TCW_CTRL_REG);

	return insn->n;
}

static int apci1564_timer_insn_write(struct comedi_device *dev,
				     struct comedi_subdevice *s,
				     struct comedi_insn *insn,
				     unsigned int *data)
{
	struct apci1564_private *devpriv = dev->private;
	unsigned int ctrl;

	ctrl = inl(devpriv->timer + ADDI_TCW_CTRL_REG);
	switch (data[1]) {
	case 0:	/* Stop The Timer */
		ctrl &= 0xfffff9fe;
		break;
	case 1:	/* Enable the Timer */
		ctrl &= 0xfffff9ff;
		ctrl |= 0x1;
		break;
	}
	outl(ctrl, devpriv->timer + ADDI_TCW_CTRL_REG);

	return insn->n;
}

static int apci1564_timer_insn_read(struct comedi_device *dev,
				    struct comedi_subdevice *s,
				    struct comedi_insn *insn,
				    unsigned int *data)
{
	struct apci1564_private *devpriv = dev->private;

	/* Stores the status of the Timer */
	data[0] = inl(devpriv->timer + ADDI_TCW_STATUS_REG) & 0x1;

	/* Stores the Actual value of the Timer */
	data[1] = inl(devpriv->timer + ADDI_TCW_VAL_REG);

	return insn->n;
}

static int apci1564_counter_insn_config(struct comedi_device *dev,
					struct comedi_subdevice *s,
					struct comedi_insn *insn,
					unsigned int *data)
{
	struct apci1564_private *devpriv = dev->private;
	unsigned int chan = CR_CHAN(insn->chanspec);
	unsigned long iobase = devpriv->counters + APCI1564_COUNTER(chan);
	unsigned int ctrl;

	devpriv->tsk_current = current;

	/* First Stop The Counter */
	ctrl = inl(iobase + ADDI_TCW_CTRL_REG);
	ctrl &= 0xfffff9fe;
	/* Stop The Timer */
	outl(ctrl, iobase + ADDI_TCW_CTRL_REG);

	/* Set the reload value */
	outl(data[3], iobase + ADDI_TCW_RELOAD_REG);

	/* Set the mode :             */
	/* - Disable the hardware     */
	/* - Disable the counter mode */
	/* - Disable the warning      */
	/* - Disable the reset        */
	/* - Disable the timer mode   */
	/* - Enable the counter mode  */

	ctrl &= 0xfffc19e2;
	ctrl |= 0x80000 | (data[4] << 16);
	outl(ctrl, iobase + ADDI_TCW_CTRL_REG);

	/* Enable or Disable Interrupt */
	ctrl &= 0xfffff9fd;
	ctrl |= (data[1] << 1);
	outl(ctrl, iobase + ADDI_TCW_CTRL_REG);

	/* Set the Up/Down selection */
	ctrl &= 0xfffbf9ff;
	ctrl |= (data[6] << 18);
	outl(ctrl, iobase + ADDI_TCW_CTRL_REG);

	return insn->n;
}

static int apci1564_counter_insn_write(struct comedi_device *dev,
				       struct comedi_subdevice *s,
				       struct comedi_insn *insn,
				       unsigned int *data)
{
	struct apci1564_private *devpriv = dev->private;
	unsigned int chan = CR_CHAN(insn->chanspec);
	unsigned long iobase = devpriv->counters + APCI1564_COUNTER(chan);
	unsigned int ctrl;

	ctrl = inl(iobase + ADDI_TCW_CTRL_REG);
	switch (data[1]) {
	case 0:	/* Stops the Counter subdevice */
		ctrl = 0;
		break;
	case 1:	/* Start the Counter subdevice */
		ctrl &= 0xfffff9ff;
		ctrl |= 0x1;
		break;
	case 2:	/* Clears the Counter subdevice */
		ctrl &= 0xfffff9ff;
		ctrl |= 0x400;
		break;
	}
	outl(ctrl, iobase + ADDI_TCW_CTRL_REG);

	return insn->n;
}

static int apci1564_counter_insn_read(struct comedi_device *dev,
				      struct comedi_subdevice *s,
				      struct comedi_insn *insn,
				      unsigned int *data)
{
	struct apci1564_private *devpriv = dev->private;
	unsigned int chan = CR_CHAN(insn->chanspec);
	unsigned long iobase = devpriv->counters + APCI1564_COUNTER(chan);
	unsigned int status;

	/* Read the Counter Actual Value. */
	data[0] = inl(iobase + ADDI_TCW_VAL_REG);

	status = inl(iobase + ADDI_TCW_STATUS_REG);
	data[1] = (status >> 1) & 1;	/* software trigger status */
	data[2] = (status >> 2) & 1;	/* hardware trigger status */
	data[3] = (status >> 3) & 1;	/* software clear status */
	data[4] = (status >> 0) & 1;	/* overflow status */

	return insn->n;
}
