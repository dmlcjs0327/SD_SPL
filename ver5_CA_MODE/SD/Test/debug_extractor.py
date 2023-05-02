import re, os, charset_normalizer

file_name = r"C:\LUC\jupyter\2023_lab_product_line\SD_SPL\ver5_CA_MODE\SD\Plan\Planner5.py"
file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)


with open(file_path, "rb") as f:
    raw_data = f.read()

encoding = charset_normalizer.detect(raw_data)["encoding"]
print(encoding)

with open(file_path, "r", encoding=encoding) as f:
    content = f.read()

pattern = r'def\s+\w+\(.*?\):'
# pattern = r'def test_TID+\w+\(.*?\):'
# 정규식 패턴에 매칭되는 모든 문자열 추출
result = re.findall(pattern, content, re.DOTALL)
result = list(map(lambda x: x[4:-1],result))

# 결과 출력
for i in result:
    print(i)
print(len(result))