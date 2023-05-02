import re, os, chardet

file_name = "TEST_run2_tof.py"
file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)


with open(file_path, "rb") as f:
    raw_data = f.read()

encoding = chardet.detect(raw_data)["encoding"]
print(encoding)

with open(file_path, "r", encoding=encoding) as f:
    content = f.read()

pattern = r"(test_TID0)(\d{3})"
matches = re.findall(pattern, content)

re_obj = re.compile(pattern)  # 정규표현식을 컴파일한 re 객체 생성

pre=0
for i in range(len(matches)):
    match = re_obj.search(content, pos=pre)
    if match:
        target = "".join(match.group())
        new_string = f"test_TID{i+1001:04d}"
        content = content[:match.start()] + new_string + content[match.end():]
        pre = match.end()
        # print("t:",target, "n:",new_string)
        # print(content)
        # break
with open(file_path, "w", encoding="utf-8") as f:
    f.write(content)

print("완료")