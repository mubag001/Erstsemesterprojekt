#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "utility.hpp"
// Enthält die notwendigen Includes für die Entwicklung mit der depthai-Bibliothek
#include "depthai/depthai.hpp"

using namespace cv;

// Optional. Wenn auf (true) gesetzt, wird die ColorCamera von 1080p auf 720p herunterskaliert.
// Andernfalls (false) wird die ausgerichtete Tiefe automatisch auf 1080p hochskaliert.
static std::atomic<bool> downscaleColor{true};
static constexpr int fps = 30;
// Die Disparität wird in dieser Auflösung berechnet und dann auf RGB-Auflösung hochskaliert
static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_720_P;

static float rgbWeight = 0.4f;
static float depthWeight = 0.6f;

static void updateBlendWeights(int percentRgb, void* ctx) {
    rgbWeight = float(percentRgb) / 100.f;
    depthWeight = 1.f - rgbWeight;
}

static int schwellwert( Mat &img);
static String binaryImg(Mat &img, int schwelle);
static String auswertung(Mat& binary, Mat& depth);

int main(int argc, char* argv[]) {
    using namespace std;

    if(*argv[1] == '1' || *argv[1] == '2') {
        // Erstellen der Pipeline
        dai::Pipeline pipeline;
        dai::Device device;
        std::vector<std::string> queueNames;

        // Definieren der Quellen und Ausgänge
        auto camRgb = pipeline.create<dai::node::ColorCamera>();
        auto left = pipeline.create<dai::node::MonoCamera>();
        auto right = pipeline.create<dai::node::MonoCamera>();
        auto stereo = pipeline.create<dai::node::StereoDepth>();

        //Konfidenzschwellwert
        stereo->initialConfig.setConfidenceThreshold(200);

        auto rgbOut = pipeline.create<dai::node::XLinkOut>();
        auto depthOut = pipeline.create<dai::node::XLinkOut>();

        rgbOut->setStreamName("rgb");
        queueNames.push_back("rgb");
        depthOut->setStreamName("depth");
        queueNames.push_back("depth");

        // Eigenschaften
        camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
        camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        camRgb->setFps(fps);
        if (downscaleColor) camRgb->setIspScale(2, 3);

        // Für die richtige Ausrichtung der Tiefe ist ein fixer Fokus für RGB erforderlich.
        try {
            auto calibData = device.readCalibration2();
            auto lensPosition = calibData.getLensPosition(dai::CameraBoardSocket::CAM_A);
            if (lensPosition) {
                camRgb->initialControl.setManualFocus(lensPosition);
            }
        } catch (const std::exception &ex) {
            std::cout << ex.what() << std::endl;
            return 1;
        }

        left->setResolution(monoRes);
        left->setCamera("left");
        left->setFps(fps);
        right->setResolution(monoRes);
        right->setCamera("right");
        right->setFps(fps);

        stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
        //Medianfilter, Disparität und Schwellwert setzen
        stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
        stereo->setLeftRightCheck(true);    // LR-Überprüfung ist für die Tiefenausrichtung erforderlich
        stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
        stereo->setExtendedDisparity(false);

        auto config = stereo->initialConfig.get();
        config.postProcessing.thresholdFilter.minRange = 500;
        config.postProcessing.thresholdFilter.maxRange = 1000;
        stereo->initialConfig.set(config);

        // Verknüpfungen
        camRgb->isp.link(rgbOut->input);
        left->out.link(stereo->left);
        right->out.link(stereo->right);
        stereo->disparity.link(depthOut->input);

        // Verbinden mit dem Gerät und Starten der Pipeline
        device.startPipeline(pipeline);

        // Setzt die Warteschlangengröße und das Verhalten
        for (const auto &name: queueNames) {
            device.getOutputQueue(name, 4, false);
        }

        std::unordered_map<std::string, cv::Mat> frame;

        auto rgbWindowName = "rgb";
        auto depthWindowName = "depth";
        cv::namedWindow(rgbWindowName);
        cv::namedWindow(depthWindowName);

        if(*argv[1] == '1') {
            std::cout << "Live-Anzeige Modus" << std::endl;
            String ausgabe;
            while (true) {
                std::unordered_map<std::string, std::shared_ptr<dai::ImgFrame>> latestPacket;

                auto queueEvents = device.getQueueEvents(queueNames);
                for (const auto &name: queueEvents) {
                    auto packets = device.getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
                    if (!packets.empty()) {
                        latestPacket[name] = packets.back();
                    }
                }

                for (const auto &name: queueNames) {
                    if (latestPacket.find(name) != latestPacket.end()) {
                        if (name == depthWindowName) {
                            frame[name] = latestPacket[name]->getFrame();
                            auto maxDisparity = stereo->initialConfig.getMaxDisparity();
                            // Konvertieren der Tiefenbilder auf 8-Bit und Anwenden der Farbkarte
                            frame[name].convertTo(frame[name], CV_8UC1, 255.0 / maxDisparity);
                            //cv::applyColorMap(frame[name], frame[name], cv::COLORMAP_JET);
                        } else {
                            frame[name] = latestPacket[name]->getCvFrame();
                        }

                        cv::imshow(name, frame[name]);

                        //Live-Auswertung
                        if (name == depthWindowName) {
                            int schwelle = schwellwert(frame[name]);
                            //Binärbild berechnen und anzeigen
                            ausgabe = binaryImg(frame[name],schwelle);
                        } else {
                            putText(frame[name],ausgabe,Point(10,30),FONT_HERSHEY_SIMPLEX,1,Vec3b(0,0,255),2);
                            cv::imshow(name, frame[name]);
                        }
                    }
                }

                if (cv::waitKey(1) == 'q') {
                    break;
                }
            }
        }

        if(*argv[1] == '2') {
            std::cout << "Aufzeichnungsmodus" << std::endl;
            VideoWriter writerRgb;
            cv::VideoWriter writerDepth;
            int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            std::string filenameRgb = "./live_rgb.avi";
            std::string filenameDepth = "./live_depth.avi";

            writerRgb.open(filenameRgb, codec, fps, cv::Size(1280,720), true);
            writerDepth.open(filenameDepth, 0, fps, cv::Size(1280,720), false);

            if (!writerDepth.isOpened() || !writerRgb.isOpened()) {
                cerr << "Fehler beim Öffnen der Videoausgabedateien\n";
                return -3;
            }

            while (true) {
                std::unordered_map<std::string, std::shared_ptr<dai::ImgFrame>> latestPacket;

                auto queueEvents = device.getQueueEvents(queueNames);
                for (const auto &name: queueEvents) {
                    auto packets = device.getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
                    if (!packets.empty()) {
                        latestPacket[name] = packets.back();
                    }
                }

                for (const auto &name: queueNames) {
                    if (latestPacket.find(name) != latestPacket.end()) {
                        if (name == depthWindowName) {
                            frame[name] = latestPacket[name]->getFrame();
                            auto maxDisparity = stereo->initialConfig.getMaxDisparity();
                            frame[name].convertTo(frame[name], CV_8UC1, 255.0 / maxDisparity);
                            //cv::applyColorMap(frame[name], frame[name], cv::COLORMAP_JET);
                        } else {
                            frame[name] = latestPacket[name]->getCvFrame();
                        }

                        cv::imshow(name, frame[name]);

                        // RGB- und Tiefenbilder in den jeweiligen Videoausgaben speichern
                        if (name == "depth") {
                            writerDepth.write(frame[name]);
                        } else {
                            writerRgb.write(frame[name]);
                        }
                    }
                }

                if (cv::waitKey(1) == 'q') {
                    break;
                }
            }

            writerRgb.release();
            writerDepth.release();
        }
    }

    if(*argv[1] == '3') {
        // Wiedergabemodus für aufgezeichnete Videos
        cv::VideoCapture videoRgb("live_rgb.avi");
        cv::VideoCapture videoDepth("live_depth.avi");

        if (!videoDepth.isOpened() || !videoRgb.isOpened()) {
            cerr << "Fehler beim Öffnen der Videodateien\n";
            return -1;
        }

        cv::Mat frameDepth;
        Mat frameRgb;

        while (true) {
            bool successRgb = videoRgb.read(frameRgb);
            bool successDepth = videoDepth.read(frameDepth);

            if (!successDepth || !successRgb) {
                cerr << "Ende des Videos oder Fehler beim Lesen des Bildes\n";
                break;
            }

            cv::imshow("Wiedergabe Depth", frameDepth);

            //in Grauwertbild umwandeln
            cvtColor(frameDepth,frameDepth,COLOR_BGR2GRAY);

            //Histogramm berechnen und anzeigen
            int schwelle = schwellwert(frameDepth);

            //Binärbild berechnen und anzeigen
            String ausgabe = binaryImg(frameDepth,schwelle);

            //Text
            putText(frameRgb,ausgabe,Point(10,30),FONT_HERSHEY_SIMPLEX,1,Vec3b(0,0,255),2);
            cv::imshow("Wiedergabe RGB", frameRgb);

            if (cv::waitKey(30) == 'q') {
                break;
            }
        }

        videoDepth.release();
        cv::destroyAllWindows();
    }

    return 0;
}

// Funktion zur Bestimmung des Schwellwerts des Grauwert-Bilds
static int schwellwert(Mat &img){
    //Histogramm berechnen
    int hist[256];
    for(int i=0; i<256; i++){
        hist[i] = 0;
    }
    for(int i=0; i<img.rows; i++){
        for(int j=0; j<img.cols; j++){
            hist[img.at<uchar>(i,j)] += 1;
        }
    }

    //Histogramm kopieren
    int historiginal[256];
    for(int i=0; i<256; i++){
        historiginal[i] = hist[i];
    }

    //Histogrammwerte proportional verkleinern und Maximalwert bestimmen
    int maxhist = 0;
    for(int i=0; i<256; i++){
        hist[i] = (int)(hist[i]/5000);
        if(hist[i] > maxhist){
            maxhist = hist[i];
        }
    }
    //std::cout << maxhist << std::endl;

    //Histogramm glätten mit Gauß
    Mat blurred(1,256,CV_16UC1);
    for(int i=0; i<256; i++){
        blurred.at<ushort>(0,i) = hist[i];
    }
    GaussianBlur(blurred,blurred,Size(19,19),2.0);
    for(int i=0; i<256; i++){
        hist[i] = blurred.at<ushort>(0,i);
    }

    //
    //Histogrammbild berechnen
    //

    //Größe
    int x = 256; int y = maxhist;
    Mat data(y, x, CV_8UC1);

    //Schwarzer Hintergrund
    for(int i=0; i<data.rows; i++){
        for(int j=0; j<data.cols; j++){
            data.at<uchar>(i,j) = 0;
        }
    }

    for(int i=0; i<x; i++){
        int val = y - hist[i];
        //Zeichne Punkte
        if(val == y){
            data.at<uchar>(val-1, i) = 200;
        } else {
            data.at<uchar>(val, i) = 200;
        }
        //Zeichne Linien
        if(i+1 < 256) {
            int nextval = y - hist[i + 1];
            for (int j = val; j >= 0; j--) {
                if (j < y && i + 1 < 256 && nextval < j) {
                    data.at<uchar>(j, i) = 200;
                }
            }
            for (int j = val; j < y; j++) {
                if (j < y && i + 1 < 256 && nextval > j) {
                    data.at<uchar>(j, i) = 200;
                }
            }
        }
    }

    //
    //Schwellenwert finden
    //

    //größter Grauwert im Bild
    int maxg = 0;
    for(int i=255; i>=0; i--){
        if(hist[i] > maxg){
            maxg = i;
            break;
        }
    }

    //erster Histogrammwert größer 200
    int greater200 = 0;
    for(int i=maxg; i>=0; i--){
        if(hist[i] > (int)(200/5000)) {
            greater200 = i;
            break;
        }
    }

    //suche nächstes lokales Maximum
    int maxpos = 0;
    for(int i=greater200; i>=1; i--){
        if(hist[i] > hist[i-1]){
            maxpos = i;
            break;
        }
    }

    //suche nächstes lokales Minimum
    int minpos = 0;
    for(int i=maxpos; i>=1; i--){
        if(hist[i] < hist[i-1]){
            minpos = i;
            break;
        }
    }

    //Balken ins Histogramm zeichnen
    for(int i=0; i<y-1; i++){
        data.at<uchar>(i,minpos) = 150;
        data.at<uchar>(i,maxpos) = 150;
    }

    //Histogrammbild darstellen
    imshow("Histogramm", data);

    return minpos;
}

//Binärbild berechnen
static String binaryImg(Mat &img, int schwelle){
    Mat separated(img.rows, img.cols, CV_8UC1);
    for(int i=0; i<img.rows; i++){
        for(int j=0; j<img.cols; j++){
            if(img.at<uchar>(i,j) >= schwelle+5){
                separated.at<uchar>(i,j) = 255;
            } else {
                separated.at<uchar>(i,j) = 0;
            }
        }
    }

    String ausgabe = auswertung(separated, img);
    return ausgabe;
    //imshow("Binaerbild", separated);
}

int alteWerte[3] = {0,0,0};

//Auswertung
static String auswertung(Mat& binary, Mat& depth){
    //Region Labeling
    Mat labels(binary.rows, binary.cols, CV_16U);
    Mat stats, centroids;
    int objs = connectedComponentsWithStats(binary, labels, stats, centroids, 8, CV_16U);

    int maxSize = 0;
    int maxLabel = -1;
    for(int i=1; i<objs; i++){
        if(stats.at<int>(i,CC_STAT_AREA) > maxSize){
            maxSize = stats.at<int>(i,CC_STAT_AREA);
            maxLabel = i;
        }
    }

    //Rectangle
    int topY = stats.at<int>(maxLabel,CC_STAT_TOP);
    int leftX = stats.at<int>(maxLabel,CC_STAT_LEFT);
    int height = stats.at<int>(maxLabel,CC_STAT_HEIGHT);
    int width = stats.at<int>(maxLabel,CC_STAT_WIDTH);

    Mat rect(binary.rows,binary.cols,CV_8UC3);
    cvtColor(binary,rect,COLOR_GRAY2BGR);
    rectangle(rect, Point(leftX,topY), Point(leftX+width,topY+height), Vec3b(0,0,255),3);
    //imshow("Binaerbild", rect);

    //Durchschnitt der Hand im Tiefenbild berechnen
    int nmb = 0;
    double depthMeanf = 0;
    int dm = 0;
    for(int i=0; i<depth.rows; i++){
        for(int j=0; j<depth.cols; j++){
            //std::cout << depth.at<uchar>(i,j) << ", " << maxLabel << std::endl;
            if(labels.at<ushort>(i,j) == maxLabel){
                //std::cout << labels.at<ushort>(i,j) << ", " << maxLabel  << ", " << (int) (depth.at<uchar>(i,j)) << std::endl;
                //nmb++;
                depthMeanf += depth.at<uchar>(i,j);
                //std::cout << (int) (depth.at<uchar>(i,j)) << std::endl;
            }
        }
        dm += (int) (depthMeanf / depth.cols);
        //std::cout << depthMeanf << ", " << dm << std::endl;
        depthMeanf = 0;
    }
    int depthMean = (int)(dm / depth.rows);
    //std::cout << depthMean << std::endl;

    //Differenzen bilden und String erstellen
    int depthMeanOld = alteWerte[0];
    int leftXOld = alteWerte[1];
    int topYOld = alteWerte[2];

    String ausgabe = "";
    //std::cout << depthMeanf << std::endl;
    if(abs(depthMeanOld - depthMean) > 1){
        ausgabe += "vor-zuruck,";
    }
    if(abs(leftXOld - leftX) > 15){
        ausgabe += "links-rechts,";
    }
    if(abs(topYOld - topY) > 15){
        ausgabe += "hoch-runter";
    }

    //Text ins Bild schreiben
    //putText(rect,ausgabe,Point(10,30),FONT_HERSHEY_SIMPLEX,1,Vec3b(0,0,255),2);
    imshow("Binaerbild", rect);

    //Werte merken
    alteWerte[0] = depthMean;
    alteWerte[1] = leftX;
    alteWerte[2] = topY;

    return ausgabe;
}
