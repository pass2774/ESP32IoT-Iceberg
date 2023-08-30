void parse(String sData, char cSeparator)
{	
	int nCount = 0;
	int nGetIndex = 0 ;
 
	//임시저장
	String sTemp = "";
 
	//원본 복사
	String sCopy = sData;
 
	while(true)
	{
		//구분자 찾기
		nGetIndex = sCopy.indexOf(cSeparator);
 
		//리턴된 인덱스가 있나?
		if(-1 != nGetIndex)
		{
			//있다.
 
			//데이터 넣고
			sTemp = sCopy.substring(0, nGetIndex);
 
			Serial.println( sTemp );
		
			//뺀 데이터 만큼 잘라낸다.
			sCopy = sCopy.substring(nGetIndex + 1);
		}
		else
		{
			//없으면 마무리 한다.
			Serial.println( sCopy );
			break;
		}
 
		//다음 문자로~
		++nCount;
	}
 
}
