std::string nowtime()
{
    //現在時刻取得
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);

    // 表示用文字列の生成（YYYY/MM/DD_hh:mm:ss）
    const std::string s = (boost::format("%04d/%02d/%02d_%02d:%02d:%02d") % (pnow->tm_year + 1900) % (pnow->tm_mon + 1) % (pnow->tm_mday) % (pnow->tm_hour) % (pnow->tm_min) % (pnow->tm_sec)).str();

    return s;
}