#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QFileDialog>
#include <QByteArray>
#include <QThread>

#if defined(Q_OS_LINUX)

#include <stdio.h>
#include <unistd.h> // close
#include <fcntl.h>  // open
#include <time.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#endif

#define SPIFLASH_BAUDRATE			8000
#define SPIFLASH_MANUFACTURER		0x0B
#define SPIFLASH_MEMORY_TYPE		0x40
#define SPIFLASH_MEMORY_CAPACITY	0x16
#define SPIFLASH_SECTOR_COUNT		1024
#define SPIFLASH_SECTOR_SIZE		4096
#define SPIFLASH_PAGE_SIZE			256

#define SPIFLASH_TBP1_TYPICAL	20		// Byte program time (first byte) (us)
#define SPIFLASH_TBP1_MAXIMUM	50		// Maximum byte program time (first byte) (us)
#define SPIFLASH_TPP_TYPICAL	700		// Page program time (us)
#define SPIFLASH_TPP_MAXIMUM	3000	// Maximum page program time (us)
#define SPIFLASH_TSE_TYPICAL	30000	// Sector erase time (4KB) (us)
#define SPIFLASH_TSE_MAXIMUM	400000	// Maximum sector erase time (us)
#define SPIFLASH_TBE1_TYPICAL	120000	// Block erase time (32KB) (us)
#define SPIFLASH_TBE1_MAXIMUM	800000	// Maximum block erase time (32KB) (us)
#define SPIFLASH_TBE2_TYPICAL	150000	// Block erase time (64KB) (us)
#define SPIFLASH_TBE2_MAXIMUM	1000000	// Maximum block erase time (64KB) (us)
#define SPIFLASH_TCE_TYPICAL	3000000		// Chip erase time (us)
#define SPIFLASH_TCE_MAXIMUM	10000000	// Maximum chip erase time (us)

#define SPIFLASH_PAGE_PROGRAM               0x02
#define SPIFLASH_READ                       0x03
#define SPIFLASH_READ_STATUS_REGISTER       0x05
#define SPIFLASH_READ_STATUS_REGISTER2      0x35
#define SPIFLASH_WRITE_ENABLE               0x06
#define SPIFLASH_FAST_READ                  0x0B
#define SPIFLASH_SECTOR_ERASE               0x20    //0x7C//
#define SPIFLASH_BLOCK_ERASE_32KB           0x52
#define SPIFLASH_BLOCK_ERASE_64KB           0xD8
#define SPIFLASH_CHIP_ERASE                 0x60
#define SPIFLASH_DEEP_POWER_DOWN            0xB9
#define SPIFLASH_RELEASE_POWER_DOWN         0xAB
#define SPIFLASH_READ_MANUFACTURER_ID       0x90
#define SPIFLASH_READ_JEDEC_ID              0x9F
#define SPIFLASH_READ_UNIQUE_ID             0x4B

#define SPIFLASH_STATUS_BUSY                0x01
#define SPIFLASH_STATUS_WRITE_ENABLE        0x02


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class SpiFlashing : public QThread
{
    Q_OBJECT
public:
    enum Mode {
        MODE_NONE,
        MODE_READ,
        MODE_WRITE,
        MODE_ERASE,
        MODE_COMPARE,
    };

    SpiFlashing(QObject *parent = Q_NULLPTR) :
        QThread(parent)
    {
        flash_mode = MODE_NONE;
        fd_spi = -1;
        do_cancel = false;
    }
    ~SpiFlashing()
    {
        this->terminate();
    }

    void set_fd(int fd)
    {
        fd_spi = fd;
    }
    void set_param(quint8 b, quint32 s)
    {
        bits = b;
        speed = s;
    }
    void cancel()
    {
        do_cancel = true;
    }
    bool startSpiFlashErase(int addr, int length);
    bool startSpiFlashWrite(int addr, char * buff, int length);
    bool startSpiFlashRead(int addr, char * buff, int length);
    bool startSpiFlashComare(int addr, char * buff, int length);

    Mode    flash_mode;

signals:
    void set_progress(int current);
    void finished();
    void canceled();
    void compare_error();
    void add_hexlog(int start_addr, char *data, long size);
    void add_hexlog_error(int start_addr, char *read, char *ref, long size);

private Q_SLOTS:
    void run();

private:
    int     fd_spi;
    int     target_addr;
    char *  target_buff;
    int     target_length;
    quint8  bits;
    quint32 speed;
    bool    do_cancel;

    bool transfetSpi(char * write_buff, char * read_buff, int length);
    bool spiFlashWriteEnable();
    bool spiFlashExecuteCmd(char cmd, int addr);
    bool spiFlashErase(int addr, int length);
    bool spiFlashWrite(int addr, char * buff, int length);
    bool spiFlashRead(int addr, char * buff, int length);

};

class I2cWork : public QThread
{
    Q_OBJECT
public:
    enum Mode {
        MODE_NONE,
        MODE_RW,
        MODE_CHECK
    };

    I2cWork(QObject *parent = Q_NULLPTR) :
        QThread(parent)
    {
        i2c_mode = MODE_NONE;
        fd_i2c = -1;
        do_cancel = false;
    }
    ~I2cWork()
    {
        this->terminate();
    }

    void set_fd(int fd)
    {
        fd_i2c = fd;
    }
    void set_mode(Mode mod)
    {
        i2c_mode = mod;
    }
    void cancel()
    {
        do_cancel = true;
    }
    bool startI2cCheck(int fd);
    bool startI2cFlashRW(int fd, int id, QStringList &list);

signals:
    void set_check(int row, int col, bool exist);
    void finished();
    void canceled();
    void set_i2c_log(const QString &);

private Q_SLOTS:
    void run();

private:
    Mode    i2c_mode;
    bool    do_cancel;
    int     fd_i2c;
    int     slave_id;
    QStringList i2cList;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void onProgress(int current);
    void onFinished();
    void onCanceled();
    void onCompareError();
    void onAppendLog(int start_addr, char *data, long size);
    void onAppendLog_error(int start_addr, char *read, char *ref, long size);

    void onI2cChecked(int row, int col, bool exist);
    void onI2cWorkFinished();
    void onI2CworkCanceled();
    void onI2cLog(const QString &message);

private slots:
    void on_rbTx2SPI_clicked(bool checked);

    void on_rbOrinSPI_clicked(bool checked);

    void on_btnCustomWrite_clicked();

    void on_btnRead_clicked();

    void on_btnErase_clicked();

    void on_btnOpen_clicked();

    void on_btnClose_clicked();

    void on_btnSpiTestExecute_clicked();

    void on_btnClear_clicked();

    void on_btnI2cDevOpen_clicked();

    void on_btnI2cAddrCheck_clicked();

    void on_btnI2cAddrClear_clicked();

    void on_btnI2cUnitExecute_clicked();

    void on_rbWriteI2C_clicked(bool checked);

    void on_rbReadI2C_clicked(bool checked);

    void on_btnI2cSeqExecute_clicked();

    void on_btnAddI2CData_clicked();

    void on_btnRemoveI2CData_clicked();

    void on_btnSaveI2CData_clicked();

    void on_btnLoadI2CData_clicked();

    void on_btnCustomCompare_clicked();

    void on_btnCustomFileOpen_clicked();

    void on_btnCancelthread_clicked();

    void on_btnShowFlashCompareLog_clicked(bool checked);

    void on_btnSave_clicked();

    void on_btnReadManufacturerId_clicked();

    void on_btnReadJedecId_clicked();

    void on_btnReadUniqueId_clicked();

    void on_btnWriteFile_clicked();

    void on_cbDebug_clicked(bool checked);

    void on_sbSpiRWUnit_valueChanged(int arg1);

private:
    Ui::MainWindow *ui;

    // SPI working
    SpiFlashing m_flashThread;
    QByteArray m_writeBuff;
    QByteArray m_compareBuff;
    char * m_readBuff;
    int m_readBuffLen;

    int     fd_spi;
    quint32 mode;
    quint8  bits;
    quint32 speed;

    // I2C working
    int m_i2cFd;
    I2cWork m_i2cWork;


    void activateUI();
    void deactivateUI();
    void uiUpdateHexaView(int start_addr, const char *data, qint64 size);
    bool transfetSpi(char * write_buff, char * read_buff, int length);
};

#endif // MAINWINDOW_H
