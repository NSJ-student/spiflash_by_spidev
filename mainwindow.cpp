#include "mainwindow.h"
#include "ui_mainwindow.h"

/*
 * Jetson Orin SPI spinmap manual setting
busybox devmem 0x0243d008 32 0x400
busybox devmem 0x0243d018 32 0x458
busybox devmem 0x0243d028 32 0x400
busybox devmem 0x0243d038 32 0x400
busybox devmem 0x0243d040 32 0x400
*/

#if defined(Q_OS_LINUX)
#include <unistd.h> // close
#include <fcntl.h>  // open
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#endif

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->tab_flash->setEnabled(false);
    ui->gbSpiTest->setEnabled(false);
    ui->tabWidget->setCurrentIndex(0);

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
    connect(&m_flashThread, SIGNAL(compare_error()),
            this, SLOT(onCompareError()));
    connect(&m_flashThread, SIGNAL(add_hexlog(int, char*,long)),
            this, SLOT(onAppendLog(int,char*,long)));
    connect(&m_flashThread, SIGNAL(add_hexlog_error(int,char*,char*,long)),
            this, SLOT(onAppendLog_error(int,char*,char*,long)));

    QHeaderView* header = ui->tblI2cDevCheck->horizontalHeader();
    for(int cnt=0; cnt<16; cnt++)
    {
        header->setSectionResizeMode(cnt,QHeaderView::ResizeToContents);
    }
    header = ui->tblI2cDevCheck->verticalHeader();
    header->setVisible(true);

    ui->tabWidgetI2cAction->setCurrentIndex(0);
    ui->tabWidgetI2cAction->setEnabled(false);

    connect(&m_i2cWork, SIGNAL(set_check(int,int,bool)),
            this, SLOT(onI2cChecked(int,int,bool)));
    connect(&m_i2cWork, SIGNAL(set_i2c_log(const QString&)),
            this, SLOT(onI2cLog(const QString&)));
    connect(&m_i2cWork, SIGNAL(canceled()),
            this, SLOT(onI2CworkCanceled()));
    connect(&m_i2cWork, SIGNAL(finished()),
            this, SLOT(onI2cWorkFinished()));

    m_i2cFd = -1;
    for(int row=0; row<8; row++)
    {
        for(int col=0; col<16; col++)
        {
            QTableWidgetItem * widget;
            widget = ui->tblI2cDevCheck->item(row, col);
            if(widget == Q_NULLPTR)
            {
                widget = new QTableWidgetItem();
                widget->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
                ui->tblI2cDevCheck->setItem(row, col, widget);
            }
        }
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}


/***********************************/


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




/***********************/
//  SPI
/***********************/

void MainWindow::on_rbTx2SPI_clicked(bool checked)
{
    if(checked)
    {
        ui->cbSpiDevice->clear();
        ui->cbSpiDevice->addItem("/dev/spidev3.0");
        ui->cbSpiDevice->addItem("/dev/spidev3.1");
    }
}

void MainWindow::on_rbOrinSPI_clicked(bool checked)
{
    if(checked)
    {
        ui->cbSpiDevice->clear();
        ui->cbSpiDevice->addItem("/dev/spidev0.0");
        ui->cbSpiDevice->addItem("/dev/spidev0.1");
        ui->cbSpiDevice->addItem("/dev/spidev1.0");
        ui->cbSpiDevice->addItem("/dev/spidev1.1");
        ui->cbSpiDevice->addItem("/dev/spidev2.0");
        ui->cbSpiDevice->addItem("/dev/spidev2.1");
    }
}

void MainWindow::on_btnCancelthread_clicked()
{
    m_flashThread.cancel();
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
        ui->progressRW->setMaximum(target_size);
        ui->progressRW->setValue(0);
        ui->pteBinaryView->clear();
        deactivateUI();
    }
}

void MainWindow::on_btnCompareFileOpen_clicked()
{
    QString path = QFileDialog::getOpenFileName(
                this, ("Load Compare BIN File"),
                Q_NULLPTR,
                "BIN (*.bin);;All Files (*.*)");

    if(path.isNull() || path.isEmpty())
    {
        return;
    }

    ui->lineComparePath->setText(path);
    QFileInfo info(path);
    ui->sbCompareSize->setValue(info.size());
}

void MainWindow::on_btnCompare_clicked()
{
    QString path = ui->lineComparePath->text();
    if(path.isNull() || path.isEmpty())
    {
        return;
    }

    QFile file(path);
    bool ret = file.open(QIODevice::ReadOnly);
    if(!ret)
    {
        qDebug() << QString("bin_c: openFile() %1").arg(path);
        return;
    }

    m_compareBuff = file.readAll();
    file.close();
    int compare_max_size = m_compareBuff.size();
    if(compare_max_size <= 0)
    {
        qDebug() << QString("bin_c: can not read raw data");
        return;
    }

    qDebug() << QString("bin_w: write %1").arg(path);

    int address = ui->sbStartReadAddr->value();
    int offset = ui->sbFileOffset->value();
    int compare_size = ui->sbCompareSize->value();
    ui->sbCompareErrorCount->setValue(0);

    if(offset+compare_size > compare_max_size)
    {
        qDebug() << QString("bin_c: out of range");
        return;
    }

    if(m_flashThread.startSpiFlashComare(address, m_compareBuff.data()+offset, compare_size))
    {
        ui->sbSize->setValue(compare_size);
        ui->progressRW->setMaximum(compare_size);
        ui->progressRW->setValue(0);
        ui->pteBinaryView->clear();
        deactivateUI();
    }

}

void MainWindow::on_btnShowFlashCompareLog_clicked(bool checked)
{
    ui->stackedWidget_flash->setCurrentIndex(checked?1:0);
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
        ui->pteBinaryView->append(converted);
        if(offset >= size)
        {
            break;
        }
    }
}

void MainWindow::on_btnClear_clicked()
{
    ui->pteBinaryView->clear();
    ui->textFlashResult->clear();
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

/***********************************/

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
//        int address = ui->sbStartAddr->value();
//        uiUpdateHexaView(address, m_writeBuff.data(), m_writeBuff.size());
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

void MainWindow::onCompareError()
{
    ui->sbCompareErrorCount->setValue(ui->sbCompareErrorCount->value()+1);
}

void MainWindow::onAppendLog(int start_addr, char *data, long size)
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

    ui->pteBinaryView->setTextColor(Qt::black);
    int offset = 0;
    int i = 0;
    for(i=0; i<row_count; i++)
    {
        int base = (start_addr&0xFFFFFFF0)+(i*16);
        QString converted;

        if((start_addr%16) == 0)
        {
            converted = converted + QStringLiteral("%1").arg(base, 8, 16, QLatin1Char('0')) + "|";
        }

        for(int cnt=0; cnt<16; cnt++)
        {
            /*
            if((i==0) && (cnt < (start_addr&0x0F)))
            {
                converted = converted + QString("   ");
            }
            else
            */
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

        if((start_addr%16) == 0)
        {
            ui->pteBinaryView->append(converted);
        }
        else
        {
            ui->pteBinaryView->insertPlainText(converted);
        }
        if(offset >= size)
        {
            break;
        }
    }
}

void MainWindow::onAppendLog_error(int start_addr, char *read, char *ref, long size)
{
    if(read == Q_NULLPTR)
    {
        return;
    }
    if(ref == Q_NULLPTR)
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

    ui->pteBinaryView->setTextColor(Qt::red);
    QString str_read;
    QString str_ref;

    int offset = 0;
    int i = 0;

    for(i=0; i<row_count; i++)
    {
        int base = (start_addr&0xFFFFFFF0)+(i*16);
        QString converted;

        if((start_addr%16) == 0)
        {
            converted = converted + QStringLiteral("%1").arg(base, 8, 16, QLatin1Char('0')) + "|";
        }

        for(int cnt=0; cnt<16; cnt++)
        {
            QString byte_read = QString(" %1")
                    .arg(QString::number(static_cast<uchar>(read[offset]), 16)
                         .rightJustified(2, '0'));
            QString byte_ref = QString(" %1")
                    .arg(QString::number(static_cast<uchar>(ref[offset]), 16)
                         .rightJustified(2, '0'));

            converted = converted + byte_read;
            str_read = str_read + byte_read;
            str_ref = str_ref + byte_ref;

            offset++;

            if(offset >= size)
            {
                break;
            }
        }

        if((start_addr%16) == 0)
        {
            ui->pteBinaryView->append(converted);
        }
        else
        {
            ui->pteBinaryView->insertPlainText(converted);
        }
        if(offset >= size)
        {
            break;
        }
    }

    QString log = QString("compare error: addr=0x%1/read=0x%2/ref=%3")
            .arg(start_addr,8,16,QChar('0'))
            .arg(str_read.toUpper())
            .arg(str_ref.toUpper());

    ui->textFlashResult->append(log);
}




/***********************/
//  I2C
/***********************/

void MainWindow::on_btnI2cDevOpen_clicked()
{
#if defined(Q_OS_LINUX)
    if(ui->tabWidgetI2cAction->isEnabled())
    {
        ::close(m_i2cFd);

        m_i2cFd = -1;
        ui->tabWidgetI2cAction->setEnabled(false);
        ui->btnI2cDevOpen->setText("Open");
    }
    else
    {
        const char * devName = ui->cbI2cDev->currentText().toStdString().data();
        m_i2cFd = open(devName, O_RDWR);
        if (m_i2cFd < 0)
        {
            qDebug("Can't open file %s: %s\r\n",
                   devName, strerror(errno));
            return;
        }

        ui->tabWidgetI2cAction->setEnabled(true);
        ui->btnI2cDevOpen->setText("Close");
    }
#endif
}

void MainWindow::on_btnI2cAddrCheck_clicked()
{
    if(m_i2cFd < 0)
    {
        qDebug() << "invalid i2d fd";
        return;
    }

#if defined(Q_OS_LINUX)
    if(m_i2cWork.isRunning())
    {
        qDebug() << "i2c working";
        return;
    }

    this->setEnabled(false);
    if(!m_i2cWork.startI2cCheck(m_i2cFd))
    {
        this->setEnabled(true);
    }
#endif
}

void MainWindow::on_btnI2cAddrClear_clicked()
{
    for(int addr=0; addr<128; addr++)
    {
        int row = addr/16;
        int col = addr%16;
        QTableWidgetItem * widget;
        widget = ui->tblI2cDevCheck->item(row, col);
        if(widget)
        {
            widget->setText("");
        }
    }
}

void MainWindow::on_btnI2cUnitExecute_clicked()
{
    if(ui->rbReadI2C->isChecked())
    {
        if(m_i2cFd < 0)
        {
            qDebug() << "invalid i2d fd";
            return;
        }

#if defined(Q_OS_LINUX)
        ioctl(m_i2cFd, I2C_SLAVE, ui->sbI2cAddr->value());

        QByteArray sendData = QByteArray::fromHex(ui->lineWriteDate->text().toLatin1());
        if(sendData.count() > 0)
        {
            int rval = write(m_i2cFd, sendData.data(), sendData.count());
            if (rval < 0) {
                qDebug("Writing error! (%lX): %s\r\n",
                       ui->sbI2cAddr->value(),
                       strerror(errno));
                return;
            }
        }

        int read_count = ui->sbI2CReadCount->value();
        char read_data[read_count];
        int rval = read(m_i2cFd, read_data, read_count);
        if (rval < 0)
        {
            qDebug("Reading error! (%lX): %s\r\n",
                   ui->sbI2cAddr->value(),
                   strerror(errno));
            return;
        }

        ui->lineReadData->setText(QString(QByteArray(read_data,read_count).toHex(' ').toUpper()));
        qDebug() << "i2c read done";
#endif
    }
    else
    {
        if(m_i2cFd < 0)
        {
            qDebug() << "invalid i2d fd";
            return;
        }

#if defined(Q_OS_LINUX)
        ioctl(m_i2cFd, I2C_SLAVE, ui->sbI2cAddr->value());

        QByteArray sendData = QByteArray::fromHex(ui->lineWriteDate->text().toLatin1());
        if(sendData.count() > 0)
        {
            int rval = write(m_i2cFd, sendData.data(), sendData.count());
            if (rval < 0) {
                qDebug("Writing error! (%lX): %s\r\n",
                       ui->sbI2cAddr->value(),
                       strerror(errno));
                return;
            }

            qDebug() << "i2c write done";
        }
#endif
    }
}

void MainWindow::on_rbWriteI2C_clicked(bool checked)
{
    if(checked)
    {
        ui->widgetReadData->hide();
    }
}

void MainWindow::on_rbReadI2C_clicked(bool checked)
{
    if(checked)
    {
        ui->widgetReadData->show();
    }
}

void MainWindow::on_btnI2cSeqExecute_clicked()
{
    QStringList rwList;
    for(int cnt=0; cnt<ui->listI2cSequence->count(); cnt++)
    {
        QListWidgetItem * item = ui->listI2cSequence->item(cnt);
        rwList.append(item->text());
    }

    this->setEnabled(false);
    if(!m_i2cWork.startI2cFlashRW(m_i2cFd, ui->sbI2cAddr->value(), rwList))
    {
        this->setEnabled(true);
    }
}

void MainWindow::on_btnAddI2CData_clicked()
{
    QListWidgetItem * item = new QListWidgetItem("New");
    item->setFlags(item->flags() |  Qt::ItemIsEditable);
    if((ui->listI2cSequence->count()>0) &&
            (ui->listI2cSequence->selectedItems().count()>0))
    {
        int row = ui->listI2cSequence->row(ui->listI2cSequence->selectedItems().at(0));
        if(row >= 0)
        {
            ui->listI2cSequence->insertItem(row+1, item);
        }
        else
        {
            ui->listI2cSequence->addItem(item);
        }
    }
    else
    {
        ui->listI2cSequence->addItem(item);
    }
}

void MainWindow::on_btnRemoveI2CData_clicked()
{
    foreach(QListWidgetItem* item, ui->listI2cSequence->selectedItems())
    {
        int row = ui->listI2cSequence->row(item);
        delete ui->listI2cSequence->takeItem(row);
    }
}

void MainWindow::on_btnSaveI2CData_clicked()
{
    QString save_path =
             QFileDialog::getSaveFileName(
                this,
                ("Select Save File"),
                Q_NULLPTR,
                "I2C File (*.i2c);;All Files (*.*)");

    if(!save_path.isNull() && !save_path.isEmpty())
    {
        QFile file(save_path);
        bool ret = file.open(QIODevice::WriteOnly);
        if(!ret)
        {
            qDebug() <<  QString("i2c file open fail - %1")
                               .arg(save_path);
            return;
        }

        QTextStream saveStream(&file);

        for(int cnt=0; cnt<ui->listI2cSequence->count(); cnt++)
        {
            QListWidgetItem * item = ui->listI2cSequence->item(cnt);
            saveStream << item->text() << endl;
        }

        file.close();

        qDebug() << QString("save i2c seq complete: %1").arg(save_path);
    }
}

void MainWindow::on_btnLoadI2CData_clicked()
{
    QString filePath =
             QFileDialog::getOpenFileName(
                this,
                ("Select Open File"),
                Q_NULLPTR,
                "I2C File (*.i2c);;All Files (*.*)");

    if(!filePath.isNull() && !filePath.isEmpty())
    {
        QFile openFile(filePath);
        bool ret = openFile.open(QIODevice::ReadOnly);
        if(!ret)
        {
            qDebug() << QString("i2c file open fail - %1")
                               .arg(filePath);
            return;
        }

        QTextStream loadStream(&openFile);

        ui->listI2cSequence->clear();
        while(1)
        {
            QString line = loadStream.readLine();
            if(line.isEmpty())
            {
                break;
            }

            QListWidgetItem * item = new QListWidgetItem(line);
            item->setFlags(item->flags() |  Qt::ItemIsEditable);
            ui->listI2cSequence->addItem(item);
        }

        openFile.close();
        qDebug() << QString("i2c setting load");
    }
}

/***********************************/

void MainWindow::onI2cChecked(int row, int col, bool exist)
{
    QTableWidgetItem * widget = ui->tblI2cDevCheck->item(row, col);
    if(widget)
    {
        QString str_result = (exist)?"OO":"--";
        widget->setText(str_result);
    }
}

void MainWindow::onI2cWorkFinished()
{
    this->setEnabled(true);
}

void MainWindow::onI2CworkCanceled()
{
    this->setEnabled(true);
}

void MainWindow::onI2cLog(const QString &message)
{
    qDebug() << message;
}





/***********************/
//  SPI flashing thread
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
        qDebug("spiflash_erase: all chip\n");
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
    int total_size = length;
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
        emit set_progress(total_size-length);
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
//        qDebug("spiflash_write: 0x%X, %d\n", addr, length);
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
    char readBuff[length+4];

    memset(tempBuff, 0, length+4);
    tempBuff[0] = SPIFLASH_READ;
    tempBuff[1] = (char)(addr >> 16);
    tempBuff[2] = (char)(addr >> 8);
    tempBuff[3] = (char)(addr);

    if (length > 0) {
//        qDebug("spiflash_read: 0x%X, %d\n", addr, length);

        if(!transfetSpi(tempBuff, readBuff, length+4))
        {
            return false;
        }
        memcpy(buff, readBuff+4, length);
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
    do_cancel = false;
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
    do_cancel = false;
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
    do_cancel = false;
    this->start();
    return true;
}

bool SpiFlashing::startSpiFlashComare(int addr, char *buff, int length)
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

    flash_mode = MODE_COMPARE;
    target_addr = addr;
    target_buff = buff;
    target_length = length;
    do_cancel = false;
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
        int length = 4;

        while(target_length>0)
        {
            if(do_cancel)
            {
                emit canceled();
                return;
            }
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
        int length = 4;

        while(target_length>0)
        {
            if(do_cancel)
            {
                emit canceled();
                return;
            }
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
    else if(flash_mode == MODE_COMPARE)
    {
        int done = 0;
        int address = target_addr;
        char * buff = target_buff;
        int length = 4;

        while(target_length>0)
        {
            if(do_cancel)
            {
                emit canceled();
                return;
            }
            if(length > target_length)
            {
                length = target_length;
            }

            char read_buff[length];
            if(!spiFlashRead(address, read_buff, length))
            {
                retry++;
                if(retry > 5)
                {
                    emit canceled();
                    return;
                }
                continue;
            }

            if(memcmp(buff, read_buff, length) == 0)
            {
                emit add_hexlog(address, read_buff, length);
            }
            else
            {
                emit compare_error();
                emit add_hexlog_error(address, read_buff, buff, length);
            }

            done += length;
            emit set_progress(done);

            address += length;
            buff += length;
            target_length -= length;
        }
    }

    emit finished();
}



/***********************/
//  I2C working thread
/***********************/

bool I2cWork::startI2cCheck(int fd)
{
    if(this->isRunning())
    {
        qDebug() << "i2c working";
        return false;
    }

    fd_i2c = fd;
    i2c_mode = I2cWork::MODE_CHECK;
    this->start();
    return true;
}

bool I2cWork::startI2cFlashRW(int fd, int id, QStringList &list)
{
    if(this->isRunning())
    {
        qDebug() << "i2c working";
        return false;
    }

    i2cList = list;
    fd_i2c = fd;
    i2c_mode = I2cWork::MODE_RW;
    slave_id = id;
    this->start();
    return true;
}

void I2cWork::run()
{
    if(fd_i2c<0)
    {
        emit canceled();
        return;
    }

#if defined(Q_OS_LINUX)
    if(i2c_mode == MODE_CHECK)
    {
        for(int addr=0; addr<128; addr++)
        {
            ioctl(fd_i2c, I2C_SLAVE, addr);
            char data= 0;
            int rval = write(fd_i2c, &data, 1);
            int row = addr/16;
            int col = addr%16;

            emit set_check(row, col, (rval>=0));
        }
    }
    else if(i2c_mode == MODE_RW)
    {
        ioctl(fd_i2c, I2C_SLAVE, slave_id);

        foreach(QString item, i2cList)
        {
            QStringList splt = item.split("/");
            if(splt.count()<2)
            {
                continue;
            }
            if(splt.at(0) == "w")
            {
                QByteArray sendData = QByteArray::fromHex(splt.at(1).toLatin1());
                if(sendData.count() > 0)
                {
                    int rval = write(fd_i2c, sendData.data(), sendData.count());
                    if (rval < 0) {
                        emit set_i2c_log(QString("Writing error! (%1): %2")
                               .arg(slave_id, 2, 16, QChar('0'))
                               .arg(strerror(errno)));
                        continue;
                    }
                    emit set_i2c_log(QString("Write(%1): %2")
                           .arg(slave_id, 2, 16, QChar('0'))
                           .arg(QString(sendData.toHex(' ').toUpper())));
                }
            }
            else if(splt.at(0) == "r")
            {
                bool ok;
                int read_count =  splt.at(1).toUInt(&ok, 10);
                if(!ok || (read_count<=0))
                {
                    emit set_i2c_log(QString("Read error! (%1): read count error")
                           .arg(slave_id, 2, 16, QChar('0')));
                    continue;
                }
                char read_data[read_count];
                int rval = read(fd_i2c, read_data, read_count);
                if (rval < 0) {
                    emit set_i2c_log(QString("Read error! (%1): %2")
                           .arg(slave_id, 2, 16, QChar('0'))
                           .arg(strerror(errno)));
                    continue;
                }

                emit set_i2c_log(QString("Read(%1): %2")
                       .arg(slave_id, 2, 16, QChar('0'))
                       .arg(QString(QByteArray(read_data,read_count).toHex(' ').toUpper())));
            }
            else if(splt.at(0) == "d")
            {
                bool ok;
                int delay_ms =  splt.at(1).toUInt(&ok, 10);
                if(!ok || (delay_ms<=0))
                {
                    emit set_i2c_log(QString("Delay error! (%1): delay count error")
                           .arg(slave_id, 2, 16, QChar('0')));
                    continue;
                }

                QThread::msleep(delay_ms);
                emit set_i2c_log(QString("Delay: %1").arg(delay_ms));
            }
            else
            {
                qDebug() << "invalid cmd:" << item;
            }
        }
    }
#endif

    emit finished();
}
