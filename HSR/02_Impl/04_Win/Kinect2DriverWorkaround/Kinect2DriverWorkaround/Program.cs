using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Linq;

using System.Net;
using System.Net.Sockets;

using System.IO;
using System.IO.Pipes;

using Microsoft.Kinect;

namespace Kinect2DriverWorkaround
{
    class KinectDataStreamServer
    {
        private TcpListener server;
        private TcpClient client;
        private NetworkStream stream;

        public KinectDataStreamServer(int aPort)
        {
            server = new TcpListener(IPAddress.Any, aPort);
            server.Server.NoDelay = true;
            server.Start();
             
            Console.Write("Waiting for a connection... ");
            client = server.AcceptTcpClient();
            client.NoDelay = true;
            stream = client.GetStream();
            stream.Flush();
            Console.WriteLine("Connected!");
        }

        ~KinectDataStreamServer()
        {
            client.Close();
            server.Stop();
        }

        private byte rgbImgCnt = 0;
        public void sendRgb(byte[] data)
        {
            //send(data, 0xAA);
            stream.WriteByte(0xAA);
            stream.WriteByte(0x11); // rgb
            stream.WriteByte(0xAA);
            stream.WriteByte(rgbImgCnt);
            stream.WriteByte((byte)(data.Length)); //lenght (low byte)
            stream.WriteByte((byte)(data.Length>>8));
            stream.WriteByte((byte)(data.Length>>16));
            stream.WriteByte((byte)(data.Length>>24)); // length (high byte)
            stream.Write(data, 0, data.Length);
            stream.Flush();
            ++rgbImgCnt; // whitch overflows
            Console.WriteLine("RGB image sent!");
        }

        private byte depthImgCnt = 0;
        public void sendDepth(byte[] data)
        {
            //send(data, 0xA5);
            stream.WriteByte(0xAA);
            stream.WriteByte(0x22); // depth
            stream.WriteByte(0xAA);
            stream.WriteByte(depthImgCnt);
            stream.WriteByte((byte)(data.Length)); //lenght (low byte)
            stream.WriteByte((byte)(data.Length >> 8));
            stream.WriteByte((byte)(data.Length >> 16));
            stream.WriteByte((byte)(data.Length >> 24)); // length (high byte)
            stream.Write(data, 0, data.Length);
            stream.Flush();
            ++depthImgCnt; // whitch overflows
            Console.WriteLine("Depth image sent!");
        }

        private byte irImgCnt = 0;
        public void sendIr(byte[] data)
        {
            //send(data, 0xAE);
            stream.WriteByte(0xAA);
            stream.WriteByte(0x44); // ir
            stream.WriteByte(0xAA);
            stream.WriteByte(irImgCnt);
            stream.WriteByte((byte)(data.Length)); //lenght (low byte)
            stream.WriteByte((byte)(data.Length >> 8));
            stream.WriteByte((byte)(data.Length >> 16));
            stream.WriteByte((byte)(data.Length >> 24)); // length (high byte)
            stream.Write(data, 0, data.Length);
            stream.Flush();
            ++irImgCnt; // whitch overflows
            Console.WriteLine("Ir image sent!");
        }
    }

    struct ImgIdx
    {
        public int x;
        public int y;
    }

    class Program
    {        
        // kinect sensor
        static KinectSensor sensor;
        static MultiSourceFrameReader frameReader;
        static CoordinateMapper mapper;

        static KinectDataStreamServer streamer;

        // infra red
        static FrameDescription irFd;
        static ushort[] irData;
        static byte[] irDataConverted;

        // rgb
        static FrameDescription rgbFd;
        static byte[] rgbDataConverted;
        static byte[] rgbDataAlignedDepthPing;
        static byte[] rgbDataAlignedDepthPong;

        // depth
        static FrameDescription depthFd;
        static ushort[] depthData;
        static byte[] depthDataConvertedPing;
        static byte[] depthDataConvertedPong;
        static ImgIdx[] depthToColorIdx;

        static volatile int pingPong = 0;

        static TimeSpan relTime;

        static void Main(string[] args)
        {
            // Setup data
            sensor = KinectSensor.GetDefault();

            // open multiframereader for depth, color, and ir frames
            frameReader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color);
            //frameReader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color | FrameSourceTypes.Infrared);

            // ir
            irFd = sensor.InfraredFrameSource.FrameDescription;
            irData = new ushort[irFd.LengthInPixels];
            irDataConverted = new byte[irFd.LengthInPixels];

            // rgb
            rgbFd = sensor.ColorFrameSource.FrameDescription;
            rgbDataConverted = new byte[rgbFd.LengthInPixels * 4]; //rgb + alpha

            // depth
            depthFd = sensor.DepthFrameSource.FrameDescription;
            depthData = new ushort[depthFd.LengthInPixels];
            ushort[] fakeDepthData = new ushort[depthData.Length];
            for (int j = 0; j < depthData.Length; ++j)
            {
                fakeDepthData[j] = 1000; // 1m
            }
            depthToColorIdx = new ImgIdx[fakeDepthData.Length];
            ColorSpacePoint[] depthToColorIdxTmp = new ColorSpacePoint[fakeDepthData.Length];
//          DepthSpacePoint[] colorToDepthIdx = new DepthSpacePoint[rgbFd.LengthInPixels];

            // data to send
            const int rows = 360;
            const int cols = 480;
            rgbDataAlignedDepthPing = new byte[rows * cols * 3]; // rgb only
            rgbDataAlignedDepthPong = new byte[rows * cols * 3]; // rgb only
            depthDataConvertedPing = new byte[rows * cols * 2]; // 16 bit int
            depthDataConvertedPong = new byte[rows * cols * 2]; // 16 bit int

            // setup streamer
            streamer = new KinectDataStreamServer(11000);
                
            // Start kinect sensor
            sensor.Open();
            Thread.Sleep(2500); // give kinect time to start up

            mapper = sensor.CoordinateMapper;

            // Kinect parameter
            CameraIntrinsics camParameters = mapper.GetDepthCameraIntrinsics();
            Console.Write("fx = ");
            Console.WriteLine(camParameters.FocalLengthX);
            Console.Write("fy = ");
            Console.WriteLine(camParameters.FocalLengthY);
            Console.Write("cx = ");
            Console.WriteLine(camParameters.PrincipalPointX);
            Console.Write("cy = ");
            Console.WriteLine(camParameters.PrincipalPointY);

            mapper.MapDepthFrameToColorSpace(fakeDepthData, depthToColorIdxTmp);
            for(int i = 0; i < fakeDepthData.Length; ++i)
            {
                depthToColorIdx[i].x = (int)Math.Floor(depthToColorIdxTmp[i].X + 0.5);
                depthToColorIdx[i].y = (int)Math.Floor(depthToColorIdxTmp[i].Y + 0.5);
            }
   
//            mapper.MapColorFrameToDepthSpace(fakeDepthData, colorToDepthIdx);

            // wire handler for frames arrival
            relTime = new TimeSpan();
            frameReader.MultiSourceFrameArrived += frameReader_ColorDepthFrameArrived;

            // Wait if someone tells us to die or do every five seconds something else.
            Thread.Sleep(Timeout.Infinite);

            // close everithing
            frameReader.Dispose();
            sensor.Close();
        }

        static volatile bool streamerReadyFlag = true;
        static void sendOnThread()
        {
            if (pingPong == 0)
            {
                pingPong = 1;
                streamer.sendRgb(rgbDataAlignedDepthPing);
                streamer.sendDepth(depthDataConvertedPing);
            }
            else
            {
                pingPong = 0;
                streamer.sendRgb(rgbDataAlignedDepthPong);
                streamer.sendDepth(depthDataConvertedPong);
            }
            //Thread.Sleep(1); //1s
            streamerReadyFlag = true;
        }

        static void frameReader_ColorDepthFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            //Console.WriteLine("Multi source frame received!");

            int depthWidth = 0;
            int depthHeight = 0;
 
            int colorWidth = 0;
            int colorHeight = 0;

            ushort minDepth = 0;
            ushort maxDepth = 0;

            TimeSpan depthRelTime = new TimeSpan();
            TimeSpan colorRelTime = new TimeSpan();
 
            bool framesProcessed = false;
            bool colorFrameProcessed = false;
            bool depthFrameProcessed = false;
 
            MultiSourceFrame frames = e.FrameReference.AcquireFrame();
 
            if (frames != null)
            {
                using (DepthFrame depthFrame = frames.DepthFrameReference.AcquireFrame())
                {
                    using (ColorFrame colorFrame = frames.ColorFrameReference.AcquireFrame())
                    {
                        if (depthFrame != null)
                        {
                            FrameDescription depthFrameDescription = depthFrame.FrameDescription;
                            depthWidth = depthFrameDescription.Width;
                            depthHeight = depthFrameDescription.Height;

                            minDepth = depthFrame.DepthMinReliableDistance;
                            maxDepth = depthFrame.DepthMaxReliableDistance;
                            depthRelTime = depthFrame.RelativeTime;
 
                            if ((depthWidth * depthHeight) == depthData.Length)
                            {
                                depthFrame.CopyFrameDataToArray(depthData);
                                depthFrameProcessed = true;
                            }
                        }
 
                        if (colorFrame != null)
                        {
                            FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                            colorWidth = colorFrameDescription.Width;
                            colorHeight = colorFrameDescription.Height;

                            colorRelTime = colorFrame.RelativeTime;
 
                            if ((colorWidth * colorHeight * 4) == rgbDataConverted.Length)
                            {
                                if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                                {
                                    colorFrame.CopyRawFrameDataToArray(rgbDataConverted);
                                }
                                else
                                {
                                    colorFrame.CopyConvertedFrameDataToArray(rgbDataConverted, ColorImageFormat.Bgra);
                                }
                                colorFrameProcessed = true;
                            }
                        }
                        framesProcessed = true;
                    }
                }
            }
 
            // got all frames
            if (framesProcessed && depthFrameProcessed && colorFrameProcessed)
            {
                // Console.WriteLine("Depth and Color frame received!");

                // loop over each row and column of the depth
                int depthIdxByteArray = 0;
                int newColorIndex = 0;
                byte[] rgbDataAlignedDepth;
                byte[] depthDataConverted;
                if (pingPong == 0)
                {
                    rgbDataAlignedDepth = rgbDataAlignedDepthPing;
                    depthDataConverted = depthDataConvertedPing;
                }
                else
                {
                    rgbDataAlignedDepth = rgbDataAlignedDepthPong;
                    depthDataConverted = depthDataConvertedPong;
                }

                const int offsetY = 32; // offsetY on top and offsetY on bottom (when changeing this, the resolution of the image will be cahnged. Change const ints in main!)
                const int offsetX = 16; // offsetX on the left and offsetX on the bottom (when changeing this, the resolution of the image will be cahnged. Change const ints in main!)

                for (int y = offsetY; y < (depthHeight - offsetY); ++y)
                {
                    for (int x = (depthWidth - offsetX - 1); x >= offsetX; --x)
                    {
                        // calculate index into depth array
                        int depthIndex = (y * depthWidth) + x;

                        // make sure the depth pixel maps to a valid point in color space
                        int colorX = depthToColorIdx[depthIndex].x;
                        int colorY = depthToColorIdx[depthIndex].y;

                        if ((colorX >= 0) && (colorX < colorWidth) && (colorY >= 0) && (colorY < colorHeight))
                        {
                            // calculate index into color array
                            int colorIndex = ((colorY * colorWidth) + colorX) * 4;

                            rgbDataAlignedDepth[newColorIndex++] = rgbDataConverted[colorIndex++]; //b
                            rgbDataAlignedDepth[newColorIndex++] = rgbDataConverted[colorIndex++]; //g
                            rgbDataAlignedDepth[newColorIndex++] = rgbDataConverted[colorIndex]; //r
                        }
                        else
                        {
                            rgbDataAlignedDepth[newColorIndex++] = 0; //b
                            rgbDataAlignedDepth[newColorIndex++] = 0; //g
                            rgbDataAlignedDepth[newColorIndex++] = 0; //r
                        }

                        ushort tmpDepth = depthData[depthIndex];
                        ushort depthRaw = tmpDepth >= minDepth && tmpDepth <= maxDepth ? tmpDepth : (ushort)0;
                        depthDataConverted[depthIdxByteArray++] = (byte)(depthRaw); // low byte
                        depthDataConverted[depthIdxByteArray++] = (byte)(depthRaw >> 8); // high byte
                    }
                }

                // send frames
                if (streamerReadyFlag)
                {
                    streamerReadyFlag = false;
                    Thread senderThread = new Thread(sendOnThread);
                    senderThread.Start();

                    Console.Write("Time difference between the frames is: ");
                    TimeSpan diff = depthRelTime.Duration() - colorRelTime.Duration();
                    Console.Write(Math.Abs(diff.TotalMilliseconds));
                    Console.WriteLine("ms");
                    Console.Write("Time difference between the last frames is: ");
                    TimeSpan diff2 = relTime.Duration() - colorRelTime.Duration();
                    relTime = colorRelTime;
                    Console.Write(Math.Abs(diff2.TotalSeconds));
                    Console.WriteLine("s");
                }
            }
        }

    } // end Programm

} // end name space
