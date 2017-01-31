using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace SimpleExample
{
    public partial class simpleexample : Form
    {
        MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();
        // MAVLINK解析包函数，通过这个函数从串口得到一帧的数据
        bool armed = false;
        // locking to prevent multiple reads on serial port
        // 锁定以防止多读串行端口
        object readlock = new object();
        // our target sysid 
        // 我们的系统ID目标
        byte sysid;
        // our target compid
        // 我们的电脑ID目标
        byte compid;

        public simpleexample()
        {
            InitializeComponent();
        }

        private void but_connect_Click(object sender, EventArgs e)
        {
            // if the port is open close it 
            // 如果端口是关闭的
            if (serialPort1.IsOpen)
            {
                serialPort1.Close();
                return;
            }

            // set the comport options 
            // 设置相应的选项
            serialPort1.PortName = CMB_comport.Text;// 打开串口
            serialPort1.BaudRate = int.Parse(cmb_baudrate.Text);

            // open the comport 
            // 打开串口
            serialPort1.Open();

            // set timeout to 2 seconds 
            //设置超时至2秒
            serialPort1.ReadTimeout = 2000;

            BackgroundWorker bgw = new BackgroundWorker();

            bgw.DoWork += bgw_DoWork;

            bgw.RunWorkerAsync();
        }

        void bgw_DoWork(object sender, DoWorkEventArgs e)
        {
            while (serialPort1.IsOpen)
            {
                try
                {
                    MAVLink.MAVLinkMessage packet;
                    lock (readlock)
                    {
                        // read any valid packet from the port
                        // 从端口读取任何有效的数据包
                        packet = mavlink.ReadPacket(serialPort1.BaseStream);

                        // check its valid
                        // 检查它的有效
                        if (packet == null || packet.data == null)
                            continue;
                    }

                    // check to see if its a hb packet from the comport
                    // 看看它的心跳包从表现
                    if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                    {
                        var hb = (MAVLink.mavlink_heartbeat_t)packet.data;

                        // save the sysid and compid of the seen MAV
                        // 保存系统ID和目标飞行器的计算机ID
                        sysid = packet.sysid;
                        compid = packet.compid;

                        // request streams at 2 hz
                        // 请求的数据流在2Hz
                        mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
                            new MAVLink.mavlink_request_data_stream_t()
                            {
                                req_message_rate = 2,
                                req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                                start_stop = 1,
                                target_component = compid,
                                target_system = sysid
                            });
                        // 设置mavlink数据缓冲区格式
                    }

                    // from here we should check the the message is addressed to us
                    // 从这里我们应该检查的消息是写给我们（直）
                    if (sysid != packet.sysid || compid != packet.compid)
                        continue;
                    
                    if (packet.msgid == (byte)MAVLink.MAVLINK_MSG_ID.ATTITUDE)
                        //or 
                        //或
                    //if (packet.data.GetType() == typeof(MAVLink.mavlink_attitude_t))
                    {
                        //var att = (MAVLink.mavlink_attitude_t)packet.data;// 读取一帧姿态包

                        //Console.WriteLine(att.pitch*57.2958 + " " + att.roll*57.2958);

                        var imm = (MAVLink.mavlink_attitude_t)packet.data;
                        Console.WriteLine("Roll:" + imm.roll + ";Pitch:" + imm.pitch + ";Yaw:" + imm.yaw + ";");
                    }
                }
                catch
                {
                }

                System.Threading.Thread.Sleep(1);
            }
        }

        T readsomedata<T>(byte sysid,byte compid,int timeout = 2000)// 读取数据函数
        {
            DateTime deadline = DateTime.Now.AddMilliseconds(timeout);

            lock (readlock)
            {
                // read the current buffered bytes
                // 读取当前缓冲字节
                while (DateTime.Now < deadline)
                {
                    var packet = mavlink.ReadPacket(serialPort1.BaseStream);// 读出一帧数据

                    // check its not null, and its addressed to us
                    // 检查它不是空的，它的地址给我们
                    if (packet == null || sysid != packet.sysid || compid != packet.compid)
                        continue;

                    Console.WriteLine(packet);// 打印在控制台

                    if (packet.data.GetType() == typeof (T))
                    {
                        return (T) packet.data;// 从一帧MAVLINK数据中返回T类型的数据，T类型是在调用处指定的可以是心跳包，或者姿态包等等
                    }
                }
            }

            throw new Exception("No packet match found");
        }

        private void but_armdisarm_Click(object sender, EventArgs e)
        {
            MAVLink.mavlink_command_long_t req = new MAVLink.mavlink_command_long_t();

            req.target_system = 1;
            req.target_component = 1;

            req.command = (ushort)MAVLink.MAV_CMD.COMPONENT_ARM_DISARM;

            req.param1 = armed ? 0 : 1;
            armed = !armed;
            /*
            req.param2 = p2;
            req.param3 = p3;
            req.param4 = p4;
            req.param5 = p5;
            req.param6 = p6;
            req.param7 = p7;
            */

            byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req);

            serialPort1.Write(packet, 0, packet.Length);

            try
            {
                var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
                if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED) 
                {

                }
            }
            catch 
            { 
            }
        }

        private void CMB_comport_Click(object sender, EventArgs e)
        {
            CMB_comport.DataSource = SerialPort.GetPortNames();
        }
    }
}
