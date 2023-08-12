#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->tab_flash->setEnabled(false);
    ui->gbSpiTest->setEnabled(false);

    fd_spi = -1;
    mode = 0;
    bits = 8;
    speed = 500000;
    m_readBuff = Q_NULLPTR;

    connect(&m_flashThread, SIGNAL(set_progress(int)),
            this, SLOT(onProgress(int)));
    connect(&m_flashThread, SIGNAL(finished()),
            this, SLOT(onFinished()));
    connect(&m_flashThread, SIGNAL(canceled()),
            this, SLOT(onCanceled()));
    connect(&m_flashThread, SIGNAL(add_hexlog(int, char*,long)),
            this, SLOT(onAppendLog(int,char*,long)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onProgress(int current)
{
    ui->progressRW->setValue(current);
}

void MainWindow::onFinished()
{
    qDebug() << "done";
    if(m_flashThread.flash_mode == SpiFlashing::MODE_READ)
    {
        if(m_readBuff)
        {
            delete m_readBuff;
            m_readBuff = Q_NULLPTR;
        }
    }
    else if(m_flashThread.flash_mode == SpiFlashing::MODE_WRITE)
    {
        int address = ui->sbStartAddr->value();
        uiUpdateHexaView(address, m_writeBuff.data(), m_writeBuff.size());
    }
    activateUI();
}

void MainWindow::onCanceled()
{
    if(m_readBuff)
    {
        delete m_readBuff;
        m_readBuff = Q_NULLPTR;
    }
    activateUI();
}

void MainWindow::onAppendLog(int start_addr, char *data, long size)
{
    uiUpdateHexaView(start_addr, data, size);
}

void MainWindow::activateUI()
{
    ui->tab_flash->setEnabled(true);
    ui->gbSpiTest->setEnabled(true);
}

void MainWindow::deactivateUI()
{
    ui->tab_flash->setEnabled(false);
    ui->gbSpiTest->setEnabled(false);
}

void MainWindow::on_btnWrite_clicked()
{
    QString path = QFileDialog::getOpenFileName(
                this, ("Load Write BIN File"),
                Q_NULLPTR,
                "BIN (*.bin);;All Files (*.*)");

    if(path.isNull() || path.isEmpty())
    {
        return;
    }

    QFile file(path);
    bool ret = file.open(QIODevice::ReadOnly);
    if(!ret)
    {
        qDebug() << QString("bin_w: openFile() %1").arg(path);
        return;
    }

    m_writeBuff = file.readAll();
    file.close();
    int write_size = m_writeBuff.size();
    if(write_size <= 0)
    {
        qDebug() << QString("bin_w: can not read raw data");
        return;
    }

    qDebug() << QString("bin_w: write %1").arg(path);

    int address = ui->sbStartAddr->value();

    if(m_flashThread.startSpiFlashWrite(address, m_writeBuff.data(), write_size))
    {
        ui->sbSize->setValue(write_size);
        ui->progressRW->setMaximum(write_size);
        ui->progressRW->setValue(0);
        ui->pteBinaryView->clear();
        deactivateUI();
    }

}


void MainWindow::on_btnRead_clicked()
{
    int address = ui->sbStartAddr->value();
    int target_size = ui->sbSize->value();

    m_readBuff = new char[target_size];
    if(m_flashThread.startSpiFlashRead(address, m_readBuff, target_size))
    {
        ui->progressRW->setMaximum(target_size);
        ui->progressRW->setValue(0);
        ui->pteBinaryView->clear();
        deactivateUI();
    }
    else
    {
        if(m_readBuff)
        {
            delete m_readBuff;
            m_readBuff = Q_NULLPTR;
        }
    }
}


void MainWindow::on_btnErase_clicked()
{
    int address = ui->sbStartAddr->value();
    int target_size = ui->sbSize->value();

    if(m_flashThread.startSpiFlashErase(address, target_size))
    {
        deactivateUI();
    }
}

void MainWindow::uiUpdateHexaView(int start_addr, const char *data, qint64 size)
{
    if(data == Q_NULLPTR)
    {
        return;
    }
    if(size == 0)
    {
        return;
    }

    int start_base = start_addr&0xFFFFFFF0;
    int end_base = (start_addr+size)&0xFFFFFFF0;
    int row_count = ((end_base-start_base)/16) + 1;

    int offset = 0;
    int i = 0;
    for(i=0; i<row_count; i++)
    {
        int base = (start_addr&0xFFFFFFF0)+(i*16);
        QString converted = QStringLiteral("%1").arg(base, 8, 16, QLatin1Char('0')) + "|";

        for(int cnt=0; cnt<16; cnt++)
        {
            if((i==0) && (cnt < (start_addr&0x0F)))
            {
                converted = converted + QString("   ");
            }
            else
            {
                converted = converted + QString(" %1")
                        .arg(QString::number(static_cast<uchar>(data[offset]), 16)
                             .rightJustified(2, '0'));
                offset++;
            }

            if(offset >= size)
            {
                break;
            }
        }
        ui->pteBinaryView->appendPlainText(converted);
        if(offset >= size)
        {
            break;
        }
    }
}


void MainWindow::on_btnClear_clicked()
{
    ui->pteBinaryView->clear();
}


void MainWindow::on_btnOpen_clicked()
{

#if defined(Q_OS_LINUX)
    int ret = 0;
    quint32 request;
    const char * device = ui->cbSpiDevice->currentText().toStdString().data();

    fd_spi = open(device, O_RDWR);
    if (fd_spi < 0)
    {
        qDebug() << "can't open device" << device;
        return;
    }

    /*
     * spi mode
     */
    /* WR is make a request to assign 'mode' */
    mode = 0;
    if(ui->cbCPHA->isChecked())
    {
        mode |= SPI_CPHA;
    }
    if(ui->cbCPOL->isChecked())
    {
        mode |= SPI_CPOL;
    }
    if(ui->cbLsbFirst->isChecked())
    {
        mode |= SPI_LSB_FIRST;
    }
    request = mode;
    ret = ioctl(fd_spi, SPI_IOC_WR_MODE32, &mode);
    if (ret < 0)
    {
        ::close(fd_spi);
        fd_spi = -1;
        qDebug() << "can't set spi mode";
        return;
    }

    /* RD is read what mode the device actually is in */
    ret = ioctl(fd_spi, SPI_IOC_RD_MODE32, &mode);
    if (ret < 0)
    {
        ::close(fd_spi);
        fd_spi = -1;
        qDebug() << "can't get spi mode";
        return;
    }
    /* Drivers can reject some mode bits without returning an error.
     * Read the current value to identify what mode it is in, and if it
     * differs from the requested mode, warn the user.
     */
    if (request != mode)
    {
        ::close(fd_spi);
        fd_spi = -1;
        qDebug() << "WARNING device does not support requested mode" << request;
        return;
    }

    /*
     * bits per word
     */
    bits = ui->sbSpiBpw->value();
    ret = ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret < 0)
    {
        ::close(fd_spi);
        fd_spi = -1;
        qDebug() << "can't set bits per word";
        return;
    }

    ret = ioctl(fd_spi, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret < 0)
    {
        ::close(fd_spi);
        fd_spi = -1;
        qDebug() << "can't get bits per word";
        return;
    }

    /*
     * max speed hz
     */
    speed = ui->sbSpiSpeed->value()*1000;
    ret = ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret < 0)
    {
        ::close(fd_spi);
        fd_spi = -1;
        qDebug() << "can't set max speed hz";
        return;
    }

    ret = ioctl(fd_spi, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret < 0)
    {
        ::close(fd_spi);
        fd_spi = -1;
        qDebug() << "can't get max speed hz";
        return;
    }

    qDebug("spi mode: 0x%x\n", mode);
    qDebug("bits per word: %u\n", bits);
    qDebug("max speed: %u Hz (%u kHz)\n", speed, speed/1000);
#else
    fd_spi = 1;
#endif
    m_flashThread.set_fd(fd_spi);
    m_flashThread.set_param(bits, speed);
    ui->cbSpiDevice->setEnabled(false);
    ui->cbCPHA->setEnabled(false);
    ui->cbCPOL->setEnabled(false);
    ui->cbLsbFirst->setEnabled(false);
    ui->sbSpiBpw->setEnabled(false);
    ui->sbSpiSpeed->setEnabled(false);
    ui->btnOpen->setEnabled(false);
    ui->btnClose->setEnabled(true);
    ui->tab_flash->setEnabled(true);
    ui->gbSpiTest->setEnabled(true);
}


void MainWindow::on_btnClose_clicked()
{
#if defined(Q_OS_LINUX)
    if(fd_spi >= 0)
    {
        ::close(fd_spi);
    }
#endif
    fd_spi = -1;
    m_flashThread.set_fd(fd_spi);
    ui->cbSpiDevice->setEnabled(true);
    ui->cbCPHA->setEnabled(true);
    ui->cbCPOL->setEnabled(true);
    ui->cbLsbFirst->setEnabled(true);
    ui->sbSpiBpw->setEnabled(true);
    ui->sbSpiSpeed->setEnabled(true);
    ui->btnOpen->setEnabled(true);
    ui->btnClose->setEnabled(false);
    ui->tab_flash->setEnabled(false);
    ui->gbSpiTest->setEnabled(false);
}


void MainWindow::on_btnSpiTestExecute_clicked()
{
    ui->txtSpiReadData->clear();
#if defined(Q_OS_LINUX)
    int ret;
    int out_fd;
    QByteArray sendData =
            QByteArray::fromHex(ui->txtSpiWriteData->text().toLatin1());
    int length = std::max(ui->sbSpiDataSize->value(), sendData.count());
    char writeData[length];
    char readData[length];

    if(ui->sbSpiDataSize->value() > sendData.count())
    {
        memset(writeData, ui->sbSpiDataFill->value(), length);
    }
    memcpy(writeData, sendData.data(), sendData.count());

    if(transfetSpi(writeData, readData, length))
    {
        ui->txtSpiReadData->setText(QString(QByteArray(readData, length).toHex(' ').toUpper()));
    }
#endif
}

bool MainWindow::transfetSpi(char *write_buff, char *read_buff, int length)
{
    if(fd_spi < 0)
    {
        qDebug() << "spi device not opened";
        return false;
    }
#if defined(Q_OS_LINUX)
    int ret;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)write_buff,
        .rx_buf = (unsigned long)read_buff,
        .len = length,
        .speed_hz = speed,
        .delay_usecs = 1000,
        .bits_per_word = bits,
        .cs_change = 0,
        .tx_nbits = 0,
        .rx_nbits = 0,
        .pad = 0,
    };

    if(write_buff==0)
    {
        char test[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
        tr.tx_buf = (unsigned long)test;
    }

    ret = ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0)
    {
        qDebug() << "can't send spi message";
        return false;
    }
#endif
    return true;
}



/***********************/
//
/***********************/

bool SpiFlashing::transfetSpi(char *write_buff, char *read_buff, int length)
{
    if(fd_spi < 0)
    {
        qDebug() << "spi device not opened";
        return false;
    }
#if defined(Q_OS_LINUX)
    int ret;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)write_buff,
        .rx_buf = (unsigned long)read_buff,
        .len = length,
        .speed_hz = speed,
        .delay_usecs = 1000,
        .bits_per_word = bits,
        .cs_change = 0,
        .tx_nbits = 0,
        .rx_nbits = 0,
        .pad = 0,
    };

    ret = ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0)
    {
        qDebug() << "can't send spi message";
        return false;
    }
#endif
    return true;
}

bool SpiFlashing::spiFlashWriteEnable()
{
    char write_buff = SPIFLASH_WRITE_ENABLE;
    return transfetSpi(&write_buff, 0, 1);
}

bool SpiFlashing::spiFlashExecuteCmd(char cmd, int addr)
{
    char buff[4] = { cmd, (char)(addr >> 16U), (char)(addr >> 8U),
                            (char)addr };

    return transfetSpi(buff, 0, 4);
}

bool SpiFlashing::spiFlashErase(int addr, int length)
{
    if(!spiFlashWriteEnable())
    {
        return false;
    }

    if (addr == 0 && (length == (SPIFLASH_SECTOR_COUNT*SPIFLASH_SECTOR_SIZE)))
    {
        char write_buff = SPIFLASH_CHIP_ERASE;
        bool ret = transfetSpi(&write_buff, 0, 1);
        if(ret)
        {
            QThread::usleep(SPIFLASH_TCE_MAXIMUM);
        }
        return ret;
    }

    qDebug("spiflash_erase: 0x%X, %d\n", addr, length);
    addr &= ~0xFFFU;
    while (length)
    {
        if(!spiFlashExecuteCmd(SPIFLASH_SECTOR_ERASE, addr))
        {
            return false;
        }
        QThread::usleep(SPIFLASH_TSE_MAXIMUM);
        addr += SPIFLASH_SECTOR_SIZE;
        if (length > SPIFLASH_SECTOR_SIZE)
        {
            length -= SPIFLASH_SECTOR_SIZE;
        }
        else
        {
            length = 0;
        }
    }

    return true;
}

bool SpiFlashing::spiFlashWrite(int addr, char *buff, int length)
{
    const char *u8buf;
    int page_limit;
    int to_write;
    int pp_time_typical;
    int pp_time_maximum;

    u8buf = buff;

    pp_time_typical = SPIFLASH_TBP1_TYPICAL;
    pp_time_maximum = SPIFLASH_TBP1_MAXIMUM;
    if (pp_time_maximum < pp_time_typical) {
        pp_time_maximum = pp_time_typical;
    }

    while (length) {
        qDebug("spiflash_write: 0x%X, %d\n", addr, length);
        if(!spiFlashWriteEnable())
        {
            return false;
        }

        page_limit = (addr & ~(SPIFLASH_PAGE_SIZE - 1)) + SPIFLASH_PAGE_SIZE;
        to_write = ((page_limit - addr) > length) ? length :  (page_limit - addr);

        char tempBuff[to_write+4];
        tempBuff[0] = SPIFLASH_PAGE_PROGRAM;
        tempBuff[1] = (char)(addr >> 16);
        tempBuff[2] = (char)(addr >> 8);
        tempBuff[3] = (char)(addr);
        memcpy(tempBuff+4, u8buf, to_write);
        if(!transfetSpi(tempBuff, 0, to_write+4))
        {
            return false;
        }
        /* Now we know that device is not ready */
        QThread::usleep(pp_time_typical);
        QThread::usleep(pp_time_maximum - pp_time_typical);

        addr += to_write;
        u8buf += to_write;
        length -= to_write;
    }

    return true;
}

bool SpiFlashing::spiFlashRead(int addr, char *buff, int length)
{
    char tempBuff[length+4];

    tempBuff[0] = SPIFLASH_READ;
    tempBuff[1] = (char)(addr >> 16);
    tempBuff[2] = (char)(addr >> 8);
    tempBuff[3] = (char)(addr);

    if (length > 0) {
        qDebug("spiflash_read: 0x%X, %d\n", addr, length);

        if(!transfetSpi(tempBuff, buff, length+4))
        {
            return false;
        }
    }

    return true;
}

bool SpiFlashing::startSpiFlashErase(int addr, int length)
{
    if(fd_spi < 0)
    {
        qDebug() << "spi device not opened";
        return false;
    }

    flash_mode = MODE_ERASE;
    target_addr = addr;
    target_length = length;
    this->start();
    return true;
}

bool SpiFlashing::startSpiFlashWrite(int addr, char *buff, int length)
{
    if(fd_spi < 0)
    {
        qDebug() << "spi device not opened";
        return false;
    }
    if((addr&0x0F)>0)
    {
        qDebug() << "start addr not aligned";
        return false;
    }

    flash_mode = MODE_WRITE;
    target_addr = addr;
    target_buff = buff;
    target_length = length;
    this->start();
    return true;
}

bool SpiFlashing::startSpiFlashRead(int addr, char *buff, int length)
{
    if(fd_spi < 0)
    {
        qDebug() << "spi device not opened";
        return false;
    }
    if((addr&0x0F)>0)
    {
        qDebug() << "start addr not aligned";
        return false;
    }

    flash_mode = MODE_READ;
    target_addr = addr;
    target_buff = buff;
    target_length = length;
    this->start();
    return true;
}

void SpiFlashing::run()
{
    if(fd_spi < 0)
    {
        return;
    }
    int retry = 0;

    qDebug() << "start" << target_length;
    if(flash_mode == MODE_READ)
    {
        int done = 0;
        int address = target_addr;
        char * buff = target_buff;
        int length = 1024;

        while(target_length>0)
        {
            if(length > target_length)
            {
                length = target_length;
            }
            if(!spiFlashRead(address, buff, length))
            {
                retry++;
                if(retry > 5)
                {
                    emit canceled();
                    return;
                }
                continue;
            }
            done += length;
            emit set_progress(done);
            emit add_hexlog(address, buff, length);

            qDebug() << "done" << target_length;
            address += length;
            buff += length;
            target_length -= length;
        }
    }
    else if(flash_mode == MODE_WRITE)
    {
        int done = 0;
        int address = target_addr;
        char * buff = target_buff;
        int length = 1024;

        while(target_length>0)
        {
            if(length > target_length)
            {
                length = target_length;
            }
            if(!spiFlashWrite(address, buff, length))
            {
                retry++;
                if(retry > 5)
                {
                    emit canceled();
                    return;
                }
                continue;
            }
            done += length;
            emit set_progress(done);
            emit add_hexlog(address, buff, length);

            qDebug() << "done" << target_length;
            address += length;
            buff += length;
            target_length -= length;
        }
    }
    else if(flash_mode == MODE_ERASE)
    {
        if(!spiFlashErase(target_addr, target_length))
        {
            emit canceled();
            return;
        }
    }

    emit finished();
}
