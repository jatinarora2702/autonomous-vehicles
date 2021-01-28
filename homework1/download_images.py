import os
import sys
import time

import requests
import urllib3
from bs4 import BeautifulSoup
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from urllib3.exceptions import InsecureRequestWarning

urllib3.disable_warnings(InsecureRequestWarning)

image_classes = ["black stork",
                 "cougar",
                 "English setter",
                 "English springer",
                 "grey whale",
                 "kit fox",
                 "lesser panda",
                 "porcupine",
                 "sea lion",
                 "Siberian husky"]

chrome_driver = "C:\\Users\\Jatin\\Documents\\Setups\\chromedriver.exe"


def download_google_static_images():
    options = webdriver.ChromeOptions()
    options.add_argument("--no-sandbox")
    # options.add_argument("--headless")

    try:
        browser = webdriver.Chrome(chrome_driver, options=options)
    except Exception as e:
        print(f"No found chromedriver in this environment.")
        print(f"Install on your machine. exception: {e}")
        sys.exit()

    browser.set_window_size(1280, 1024)
    for image_cls in image_classes:
        dirs = "pictures/%s/" % image_cls
        if not os.path.exists(dirs):
            os.makedirs(dirs)

        searchword = image_cls
        searchurl = "https://www.google.com/search?q=" + searchword + "&source=lnms&tbm=isch"
        browser.get(searchurl)
        time.sleep(1)

        print(f"Getting you a lot of images. This may take a few moments...")

        element = browser.find_element_by_tag_name("body")

        # Scroll down
        for i in range(5):
            element.send_keys(Keys.PAGE_DOWN)
            time.sleep(0.3)

        print(f"Reached end of page.")
        time.sleep(0.5)
        print(f"Retry")
        time.sleep(0.5)

        page_source = browser.page_source
        soup = BeautifulSoup(page_source, "lxml")
        images = soup.find_all("img")

        urls = []
        for image in images:
            try:
                url = image["data-src"]
                if not url.find("https://"):
                    urls.append(url)
            except:
                try:
                    url = image["src"]
                    if not url.find("https://"):
                        urls.append(image["src"])
                except Exception as e:
                    print(f"No found image sources.")
                    print(e)

        count = 0
        if urls:
            for url in urls:
                try:
                    res = requests.get(url, verify=False, stream=True)
                    rawdata = res.raw.read()
                    with open(os.path.join(dirs, "img_" + str(count) + ".jpg"), "wb") as f:
                        f.write(rawdata)
                        count += 1
                        if count > 50:
                            break
                except Exception as e:
                    print("Failed to write raw data.")
                    print(e)

    browser.close()


def main():
    t0 = time.time()
    download_google_static_images()
    t1 = time.time()

    total_time = t1 - t0
    print(f"\n")
    print(f"Download completed.")
    print(f"Total time is {str(total_time)} seconds.")


if __name__ == "__main__":
    main()
