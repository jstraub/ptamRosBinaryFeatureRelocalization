/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #include <briefIR.hpp>

ImageSet* loadImageSet(FeatureDatabase& fdb, LshKBM& lshKBM,
    vector<uint32_t>& imageIds, string pathToSet)
{//load/create inverted file
  ImageSet* imgSet;
  FILE *fTest=fopen(pathToSet.c_str(),"rb");
  if(!fTest){
    cout<<"Registering images - creation of inverted file"<<endl;
    imgSet=new ImageSet();
    imgSet->registerImages(lshKBM,fdb,imageIds);
    cout<<"Saving image set to "<<pathToSet<<endl;
    FILE *f=fopen(pathToSet.c_str(), "wb");
    if(!f)
      cerr<<"Cannot write image set file"<<endl;
    else {
      imgSet->save(f,lshKBM);
      fclose(f);
    }
  }else{
    cout<<"Loading image set from "<<pathToSet<<endl;
    imgSet = new ImageSet();
    bool ok = imgSet->load(fTest, lshKBM, lshKBM.getVersion()<2); // in version 0 and 1 the guid was not handled correctly ...
    fclose(fTest);
    if(!ok) return NULL;

    // Sanity check: query_db_index is where we will lookup detailed infos like
    // the file name later, when printing out matching images. This database
    // must be the same as the one used for image registration.
    // However, if the image set has been created using a subset of a
    // database (using -i0 -i4 -i6 ...), img_set->num_images will be less
    // than the number of images in the db. Without a database ID that's
    // stored in the image set, we cannot realiably check that condition.
    if(fdb.index().size() != imgSet->imageCount()) {
      cout<<"WARNING: Database is (probably) not the one used for the image set, you might need --query-db\n"
          "         Don't worry about this message if you registered a subset of a database (specifying images via -i)"<<endl;
    }
//    for(unsigned int node = 0; node < imgSet->wordCount(); node++) {
//      cout<<"@"<<node<<": ";
//      uint32_t imgCount=0;
//      DB_TFIDF* tfidf = imgSet->getWordInfos()[node].firstDB_TFIDF;
//      while(tfidf) {
//        for(int i=0; i<DB_TFIDF_SIZE; i++) {
//          if(tfidf->imgID[i] == -1)
//            break;
//          //            cout<<tfidf->imgID[i]<<"; ";
//        }
//        tfidf = tfidf->nextDB_TFIDF;
//        ++imgCount;
//      }
//      cout<<" #="<<imgCount<<"\t";
//    }
//    cout<<endl;
  }
  return imgSet;
}
